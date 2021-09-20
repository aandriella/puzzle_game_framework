import rospy
from board_state.msg import TokenMsg
from board_state.msg import BoardMsg

class Game(object):
  def __init__(self, board_size, task_length, n_max_attempt_per_token, timeout, objective, sociable,
               bn_game_state, bn_attempt, bn_agent_assistance, bn_user_react_time,
               bn_user_action):
    rospy.init_node('big_hero', anonymous=True)
    # subscriber for getting info from the board
    rospy.Subscriber("/detected_move", TokenMsg, self.get_move_event_callback)
    rospy.Subscriber("/board_status", BoardMsg, self.get_board_event_callback)
    #get the objective of the exercise from the launch file
    self.objective = objective
    self.with_feedback = sociable
    #we need a sleep in order to give the system the time to get the board info
    self.current_board = []
    rospy.sleep(2)
    self.initial_board = self.get_board_event()
    self.task_length = task_length
    self.n_max_attempt_per_token = n_max_attempt_per_token
    self.solution = self.set_objective(self.task_length)
    self.timeout = timeout
    self.agent_assistance = 0
    self.agent_feedback = 0
    self.avg_agent_assistance_per_move = 0
    self.outcome = 0
    self.width = board_size[0]
    self.height = board_size[1]
    #counters
    self.n_attempt_per_token = 1
    self.n_timeout_per_token = 0
    self.n_tot_sociable = 0
    self.n_sociable_per_token = 0
    self.n_mistakes = 0
    self.n_correct_move = 0
    #subscriber variables from detect_move
    self.detected_token = []
    self.picked = False
    self.placed = False
    self.moved_back = False
    #logger
    self.react_time_per_token_spec_t0 = 0.0
    self.react_time_per_token_gen_t0 = 0.0
    self.react_time_per_token_spec_t1 = 0.0
    self.react_time_per_token_gen_t1 = 0.0
    self.elapsed_time_per_token_gen_t0 = 0.0
    self.elapsed_time_per_token_spec_t0 = 0.0
    self.elapsed_time_per_token_gen_t1 = 0.0
    self.elapsed_time_per_token_spec_t1 = 0.0
    self.total_elapsed_time = 0.0
    self.move_info_gen = dict()
    self.move_info_spec = dict()
    self.move_info_summary = dict()
    self.bn_variables = dict()
    self.move_info_gen_vect = list()
    self.move_info_spec_vect = list()
    self.move_info_summary_vect = list()
    self.bn_variables_vect = list()

    self.bn_user_action = bn_user_action
    self.bn_user_react_time = bn_user_react_time
    self.bn_game_state = bn_game_state
    self.bn_attempt = bn_attempt
    self.bn_agent_assistance = bn_agent_assistance


  def get_board_event_callback(self, msg):
    '''callback from the topic board_status to get the status of the current_board'''
    self.current_board = msg.data

  def get_board_event(self):
    '''This method returns what is listened by the subscriber'''
    return self.current_board

  def get_move_event_callback(self, msg):
    '''callback from the topic detected_move to get the detected move if so'''
    if msg.detected_token == []:
      self.detected_token = ("", 0, 0)
    else:
      self.detected_token =(msg.detected_token[0], int(msg.detected_token[1]), int(msg.detected_token[2]))
    self.picked = msg.picked
    self.placed = msg.placed
    self.moved_back = msg.moved_back

  def get_move_event(self):
    '''This method just returns what is listened by the subscriber'''
    return self.detected_token, self.picked, self.placed, self.moved_back

  def check_board(self):
    '''this method checks if all the tokens are on the board if notask the user to place it correctly'''
    n_tokens_on_board = self.current_board.count('0')
    if n_tokens_on_board != self.task_length:
      print("ask the caregiver to do something")
      return False
    else:
      return True

  def check_unexpected_move(self):
    solution_so_far = self.solution[:self.n_correct_move]
    if solution_so_far!=[]:
      possible_tokens = list(set(self.initial_board)-set(["0"])-set(solution_so_far))
    else:
      possible_tokens = list(set(self.initial_board)-set(["0"]))
    if any(elem in possible_tokens for elem in self.get_board_event()[0:5]):
      print("Something happens a token has been moved...")
      token_id = self.get_board_event()[self.n_correct_move]
      self.detected_token = (token_id, self.get_token_init_loc(token_id)[1], self.n_correct_move+1)
      self.react_time_per_token_spec_t1 = 0
      return True
    else:
      return False

  def map_user_action(self, outcome):
    if outcome == -1:
      return self.bn_user_action['wrong']
    elif outcome == 0:
      return self.bn_user_action['timeout']
    elif outcome == 1:
      return self.bn_user_action['correct']


  def map_user_react_time(self):
    '''discretise the react time in 3 bins [slow, normal, fast] 0, 1a, 2
    Args:
    Return:
    0 if slow 1a if normal 2 if fast
    '''
    if self.react_time_per_token_spec_t1 > 0 and self.react_time_per_token_spec_t1 < self.bn_user_react_time['fast']:
      return 2
    elif self.react_time_per_token_spec_t1 >= self.bn_user_react_time['fast'] and self.react_time_per_token_spec_t1 <self.bn_user_react_time['normal']:
      return 1
    elif self.react_time_per_token_spec_t1 >= self.bn_user_react_time['normal']:
      return 0

  def map_game_state(self):
    '''get the game state : BEG, MIDDLE and END
    Return:
      the id of the game state
    '''
    if self.n_correct_move<self.bn_game_state['beg']:
      return 0
    elif self.n_correct_move>=self.bn_game_state['beg'] and self.n_correct_move<self.bn_game_state['mid']:
      return 1
    else:
      return 2


  def set_objective(self, n_token):
    '''The method return a list with the tokens ordered based on the solution of the exercise'''

    if self.objective == "morse":
      self.solution = ["M", "O", "R", "S", "E"]
    elif self.objective == "godel":
      self.solution = ["G", "O", "D", "E", "L"]
    elif self.objective == "euler":
      self.solution = ["E", "U", "L", "E", "R"]
    else:
      assert("Game is not defined contact the developer for integrating it")
      exit(-1)
    return self.solution

  #methods to access the variables outside the class
  def get_n_attempt_per_token(self):
    return self.n_attempt_per_token

  def get_n_max_attempt(self):
    return self.n_max_attempt

  def get_n_mistakes(self):
    return self.n_mistakes

  def get_n_solution(self):
    return self.n_solution

  def get_n_correct_move(self):
    return self.n_correct_move

  def reset_attempt_per_token(self):
    self.n_attempt_per_token = 0

  def set_n_attempt_per_token(self, value):
    self.n_attempt_per_token = value

  def set_n_mistakes(self, value):
    self.n_mistakes = value

  def set_n_correct_move(self, value):
    self.n_correct_move = value

  def set_n_solution(self, value):
    self.n_solution = value

  def set_n_max_attempt_per_token(self, value):
    self.n_max_attempt_per_token = value

  def get_game_state(self):
    '''
    get the game state : BEG, MIDDLE and END
    Return:
      the id of the game state
    '''
    if self.n_correct_move < self.bn_game_state['beg']:
      return 0
    elif self.n_correct_move >= self.bn_game_state['beg'] and self.n_correct_move < self.bn_game_state['mid']:
      return 1
    else:
      return 2


  def get_token_init_loc(self, token_id):
    '''This method returns the  token   initial location on the board'''

    token_from = [index for index in range(len(self.initial_board)) if self.initial_board[index] == token_id].pop() + 1
    return (token_id, token_from)

  def get_token_sol(self):
    '''This method returns the correct token to move from its initial location its final location'''
    token_id = self.solution[self.n_correct_move]
    token_from = [index for index in range(len(self.initial_board)) if self.initial_board[index] == token_id].pop()+1
    token_to = self.n_correct_move+1
    return (token_id, token_from, token_to)

  def get_token_row(self):
    '''This method returns the row where the correct token is'''
    token_id, token_from, token_to = self.get_token_sol()
    for r in range(self.height):
      for c in range(self.width):
        if token_from == c+(r*self.width)+1:
          return r+1

  def get_area(self, n=3):
    '''This method returns the subset n of tokens closed to the correct token'''
    token_id, token_from, token_to = self.get_token_sol()
    '''three cases can happen: 
    1:the solution is at the right side of the board
    2:the solution is in the middle
    3:the solution is at the left side of the board
    N.B: when getting the token from the array we need to decrease the counter of 1 as the counter 
    of the list starts from 0
    '''
    for c in range(1, self.height):
      if token_from == self.width*(c+1):
        #case 1

        tokens_subset = [(token_id, token_from), (self.current_board[token_from-3], token_from-2),
                         (self.current_board[token_from-2], token_from-1)]
        tokens_subset_no_empty = [token for token in tokens_subset if token[0] != "0"]
        return 2

      elif token_from == (self.width*c)+1:
        #case 3
        tokens_subset = [(token_id, token_from), (self.current_board[token_from], token_from+1),
                         (self.current_board[token_from+1], token_from+2)]
        tokens_subset_no_empty = [token for token in tokens_subset if token[0] != "0"]
        return 0

    #case 2
    tokens_subset = [(token_id, token_from), (self.current_board[token_from-2], token_from-1),
                     (self.current_board[token_from], token_from+1)]
    tokens_subset_no_empty = [token for token in tokens_subset if token[0] != "0"]

    return 1


  def get_subset(self, n=3):
    '''This method returns the subset n of tokens closed to the correct token'''
    token_id, token_from, token_to = self.get_token_sol()
    '''three cases can happen: 
    1:the solution is at the right side of the board
    2:the solution is in the middle
    3:the solution is at the left side of the board
    N.B: when getting the token from the array we need to decrease the counter of 1 as the counter 
    of the list starts from 0
    '''
    for c in range(1, self.height):
      if token_from == self.width*(c+1):
        #case 1

        tokens_subset = [(token_id, token_from), (self.current_board[token_from-3], token_from-2),
                         (self.current_board[token_from-2], token_from-1)]
        tokens_subset_no_empty = [token for token in tokens_subset if token[0] != "0"]
        return tokens_subset_no_empty

      elif token_from == (self.width*c)+1:
        #case 3
        tokens_subset = [(token_id, token_from), (self.current_board[token_from], token_from+1),
                         (self.current_board[token_from+1], token_from+2)]
        tokens_subset_no_empty = [token for token in tokens_subset if token[0] != "0"]
        return tokens_subset_no_empty

    #case 2
    tokens_subset = [(token_id, token_from), (self.current_board[token_from-2], token_from-1),
                     (self.current_board[token_from], token_from+1)]
    tokens_subset_no_empty = [token for token in tokens_subset if token[0] != "0"]

    return tokens_subset_no_empty


  def add_info_bn_variables(self, dict):
    self.bn_variables_vect.append(dict.copy())

  def add_info_gen_vect(self, dict):
    self.move_info_gen_vect.append(dict.copy())

  def add_info_spec_vect(self, dict):
    self.move_info_spec_vect.append(dict.copy())

  def store_info_spec(self, outcome):
    #timeout
    if outcome==2:
      self.move_info_spec['game_state'] = self.get_game_state()
      self.move_info_spec['token_id'] = ""
      self.move_info_spec['from'] = ""
      self.move_info_spec['to'] = ""
      self.move_info_spec['user_action'] = outcome
      self.move_info_spec['user_react_time'] = self.map_user_react_time()
      self.move_info_spec['agent_assistance'] = self.agent_assistance
      self.move_info_spec['react_time'] = self.timeout
      self.move_info_spec['elapsed_time'] = 0
      self.move_info_spec['attempt'] = self.n_attempt_per_token
      self.move_info_spec['agent_feedback'] = self.n_sociable_per_token
      self.move_info_spec['timeout'] = self.n_timeout_per_token
      self.add_info_spec_vect(self.move_info_spec)
    else:
      self.move_info_spec['user_action'] = outcome
      self.move_info_spec['user_react_time'] = self.map_user_react_time()
      self.move_info_spec['game_state'] = self.get_game_state()  
      self.move_info_spec['token_id'] = self.detected_token[0]
      self.move_info_spec['from'] = self.detected_token[1]
      self.move_info_spec['to'] = self.detected_token[2]
      self.move_info_spec['agent_assistance'] = self.agent_assistance
      self.move_info_spec['react_time'] = round(self.react_time_per_token_spec_t1, 3)
      self.move_info_spec['elapsed_time'] = round(self.elapsed_time_per_token_spec_t1, 3)
      self.move_info_spec['attempt'] = self.n_attempt_per_token
      self.move_info_spec['agent_feedback'] = self.n_sociable_per_token
      self.move_info_spec['timeout'] = self.n_timeout_per_token
      self.add_info_spec_vect(self.move_info_spec)
    return self.move_info_spec

  def store_info_gen(self):
    self.move_info_gen['game_state'] = self.get_game_state()  
    self.move_info_gen['token_id'] = self.detected_token[0]
    self.move_info_gen['from'] = self.detected_token[1]
    self.move_info_gen['to'] = self.detected_token[2]
    self.move_info_gen['avg_agent_assistance_per_move'] = round(
      self.avg_agent_assistance_per_move / self.n_attempt_per_token, 3)
    self.move_info_gen['cum_react_time'] = round(self.react_time_per_token_gen_t1, 3)
    self.move_info_gen['cum_elapsed_time'] = round(self.elapsed_time_per_token_gen_t1, 3)
    self.move_info_gen['attempt'] = self.n_attempt_per_token
    self.move_info_gen['timeout'] = self.n_timeout_per_token
    self.move_info_gen['n_agent_feedback'] = self.n_sociable_per_token
    self.add_info_gen_vect(self.move_info_gen)
    return self.move_info_gen

  def store_info_summary(self):
    entry_log_summary = ["n_attempt", "n_sociable", "avg_lev_assistance", "tot_react_time", "tot_elapsed_time"]
    self.move_info_summary["n_attempt"] = sum([elem['attempt'] for elem in self.move_info_gen_vect])
    self.move_info_summary["n_timeout"] = sum([elem['timeout'] for elem in self.move_info_gen_vect])
    self.move_info_summary["n_agent_feedback"] = sum([elem['n_agent_feedback'] for elem in self.move_info_gen_vect])
    self.move_info_summary["avg_lev_assistance"] = sum(
      [elem['avg_agent_assistance_per_move'] for elem in self.move_info_gen_vect]) / self.task_length
    self.move_info_summary["tot_react_time"] = sum([elem['cum_react_time'] for elem in self.move_info_gen_vect])
    self.move_info_summary["tot_elapsed_time"] = sum([elem['cum_elapsed_time'] for elem in self.move_info_gen_vect])
    return self.move_info_summary

  def store_bn_variables(self, outcome):
    self.bn_variables['game_state'] = self.map_game_state()
    self.bn_variables['attempt'] = self.n_attempt_per_token-1
    self.bn_variables['agent_assistance'] = self.agent_assistance
    self.bn_variables['user_action'] = outcome
    self.add_info_bn_variables(self.bn_variables)

    return self.bn_variables

  def reset_counters_spec(self):
    self.react_time_per_token_spec_t1 = 0
    self.react_time_per_token_spec_t0 = 0
    self.elapsed_time_per_token_spec_t1 = 0
    self.elapsed_time_per_token_spec_t0 = 0
    self.avg_agent_assistance_per_move = 0

  def reset_counters_gen(self):
    self.react_time_per_token_gen_t1 = 0
    self.react_time_per_token_gen_t0 = 0
    self.elapsed_time_per_token_gen_t1 = 0
    self.elapsed_time_per_token_gen_t0 = 0

  def reset_detected_token(self):
    self.detected_token = []
    self.picked = False
    self.placed = False
    self.moved_back = False


def main():
  pass

if __name__ == "__main__":
  main()
