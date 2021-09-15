#!/usr/bin/python
'''
This is the main class for running the entire game framework
'''
# import modules and classes
from log import Log
from cognitive_game import Game
from robot_behaviour.robot_reproducer import Robot
from robot_behaviour.speech_reproducer import Speech
from robot_behaviour.face_reproducer import Face
from robot_behaviour.gesture_reproducer import Gesture
# import from libraries
import enum
import random
import itertools

import time
import os
from termcolor import colored
import pygame

# import from ros
import rospy

#path in order to play audios from sounds folder
path = os.path.abspath(__file__)
dir_path = os.path.dirname(path)
parent_dir_of_file = os.path.dirname(dir_path)
parent_parent_dir_of_file = os.path.dirname(parent_dir_of_file)



class StateMachine(enum.Enum):

  def __init__(self, e):
    self.b_agent_assist_finished = False
    self.b_agent_feedback_finished = False
    self.b_user_picked_token = False
    self.b_user_placed_token_back = False
    self.b_user_placed_token_sol = False
    self.b_agent_outcome_finised = False
    self.b_agent_moved_token_back = False
    self.b_user_moved_token_back = False
    self.b_agent_moved_correct_token = False
    self.b_user_reached_max_attempt = False
    self.b_user_reached_timeout = False
    self.b_agent_reengaged_user = False

  '''States of the State machine'''
  S_ROBOT_ASSIST = 1
  S_USER_ACTION = 2
  S_USER_PICK_TOKEN = 3
  S_ROBOT_FEEDBACK = 4
  S_USER_PLACE = 5
  S_USER_PLACE_TOKEN_BACK = 6
  S_USER_PLACE_TOKEN_SOL = 7
  S_USER_MOVE_TOKEN_BACK = 8
  S_ROBOT_MOVE_TOKEN_BACK = 9
  S_ROBOT_MOVE_CORRECT_TOKEN = 10
  S_ROBOT_OUTCOME = 11
  S_USER_TIMEOUT = 12

  CURRENT_STATE = 1

  def play_sound(self, file_path, sleep):
    pygame.mixer.init()
    pygame.mixer.music.load(parent_dir_of_file+"/sounds/"+file_path)
    pygame.mixer.music.play(0)
    time.sleep(sleep)

  def stop_sound(self):
    pygame.mixer.music.stop()

  def update_counters(self, game):
    '''
        Auxiliary function to update the counters based on the outcome
        :param self:
        :return:
        '''
    print("Update_counters")

    if game.moved_back:
      if game.outcome == 2:
        game.n_timeout_per_token +=1

      print("token has been moved back, DO nothing")
      game.n_mistakes += 1
      game.n_attempt_per_token += 1
      game.set_n_attempt_per_token(game.n_attempt_per_token)
      game.set_n_mistakes(game.n_mistakes)

      if game.n_attempt_per_token > game.n_max_attempt_per_token:
        print("Max attempt reached")
        game.set_n_correct_move(game.get_n_correct_move() + 1)
        game.set_n_attempt_per_token(1)
        game.n_sociable_per_token = 0
        game.n_timeout_per_token = 0

    elif game.outcome == 2:
      game.n_timeout_per_token += 1
      game.n_mistakes += 1
      game.n_attempt_per_token += 1
      game.set_n_attempt_per_token(game.n_attempt_per_token)
      game.set_n_mistakes(game.n_mistakes)

      # check if the user reached his max number of attempts
      if game.n_attempt_per_token > game.n_max_attempt_per_token:
        print("Max attempt reached")
        game.set_n_correct_move(game.get_n_correct_move() + 1)
        game.set_n_attempt_per_token(1)
        game.n_sociable_per_token = 0
        game.n_timeout_per_token = 0

    # get current move and check if it is the one expeceted in the solution list
    elif game.outcome == 0:
      game.set_n_correct_move(game.get_n_correct_move() + 1)
      game.set_n_attempt_per_token(1)
      game.n_sociable_per_token = 0
      game.n_timeout_per_token = 0

    elif game.outcome == 1:
      print("wrong_solution")
      game.n_mistakes += 1
      game.n_attempt_per_token += 1
      game.set_n_attempt_per_token(game.n_attempt_per_token)
      game.set_n_mistakes(game.n_mistakes)

      # check if the user reached his max number of attempts
      if game.n_attempt_per_token > game.n_max_attempt_per_token:
        print("Max attempt reached")
        game.set_n_correct_move(game.get_n_correct_move() + 1)
        game.set_n_attempt_per_token(1)
        game.n_sociable_per_token = 0
        game.n_timeout_per_token = 0

  def agent_provide_feedback(self, game, agent):
    # agent.cancel_action()
    print("R_FEEDBACK")
    game.n_sociable_per_token += 1
    game.n_tot_sociable += 1
    game.agent_feedback = 1
    if game.detected_token[0] == game.solution[game.n_correct_move]:
      agent.action["pick"].__call__(positive=True, counter=game.n_attempt_per_token - 1, facial_expression="happy",eyes_coords=(0, 0) )
    else:
      agent.action["pick"].__call__(positive=False, counter=game.n_attempt_per_token - 1, facial_expression="confused", eyes_coords=(0,-80))
    self.CURRENT_STATE = self.S_USER_PLACE
    self.agent_provided_feeback_finished = True
    return self.agent_provided_feeback_finished

  def agent_provide_assistance(self, game, agent, state_index, epsilon):
    '''
    Robot action of assistance combining speech and gesture
    Args:
      game: the game instance
      agent: agent instance to perform an action
    Return:
       True when the action has been completed
    '''
    print("ROBOT GET ACTION ",agent.get_action_state())
    if  agent.get_action_state() == 0 or agent.get_action_state() == 1:
      agent.cancel_action()


    print(colored("R_ASSISTANCE", 'green'))
    #recall all the information you may need for providing assistance
    token_sol = game.get_token_sol()
    tokens_subset = game.get_subset(3)
    token_row = game.get_token_row()
    delay_for_speech = 1
    game.agent_assistance = agent.get_irl_state_action(state_index=state_index, epsilon=epsilon)

    if game.agent_assistance == 3:
      delay_for_speech = 0
    elif game.agent_assistance == 4:
      delay_for_speech = 2.5

    rospy.sleep(1.0)
    success = agent.action["assistance"].__call__(lev_id=game.agent_assistance, row=token_row, counter=game.n_attempt_per_token-1, token=token_sol, facial_expression="neutral", eyes_coords=(0,60), tokens=tokens_subset, delay_for_speech=delay_for_speech)

    self.b_agent_assist_finished = True
    self.b_agent_reengaged_user == False
    self.CURRENT_STATE = self.S_USER_ACTION
    return self.b_agent_assist_finished

  def agent_provide_outcome(self, game, agent):
    '''
    Robot provides the user with the outcome of their move
    :param self:
    :return:
    '''
    print(colored("R_OUTCOME", 'green'))

    if game.moved_back and not self.b_user_reached_timeout:
      print(colored("token has been moved back, DO nothing", 'red'))
      game.outcome = 1
      self.CURRENT_STATE = self.S_ROBOT_ASSIST

      # check if the user reached his max number of attempts
      if game.n_attempt_per_token >= game.n_max_attempt_per_token:
        print(colored("Max attempt reached", 'red'))
        self.S_ROBOT_MOVE_CORRECT_TOKEN = True
        self.b_user_reached_max_attempt = True
        self.agent_move_correct_token(game, agent)

    elif (game.detected_token == [] or game.detected_token==()) and self.b_user_reached_timeout:
      print(colored("TIMEOUT", 'red'))
      game.outcome = 2
      self.CURRENT_STATE = self.S_ROBOT_ASSIST
      self.b_user_reached_timeout = False
      #self.play_sound("timeout_trim.mp3", 3)
      agent.action["timeout"].__call__(counter=game.n_attempt_per_token - 1, facial_expression="sad", eyes_coords=(0,-80))
      game.outcome = 2
      self.CURRENT_STATE = self.S_ROBOT_ASSIST

      # check if the user reached his max number of attempts
      if game.n_attempt_per_token >= game.n_max_attempt_per_token:
        print(colored("Max attempt reached", 'red'))
        self.S_ROBOT_MOVE_CORRECT_TOKEN = True
        self.b_user_reached_max_attempt = True
        self.agent_move_correct_token(game, agent)

    elif game.detected_token != [] and self.b_user_reached_timeout:
      print(colored("TIMEOUT", 'red'))
      game.outcome = 2
      #self.play_sound("timeout_trim.mp3", 3)
      agent.action["timeout"].__call__(counter=game.n_attempt_per_token - 1, facial_expression="sad", eyes_coords=(0,-80))
      self.CURRENT_STATE = self.S_USER_MOVE_TOKEN_BACK
      self.user_move_back(game, agent)
      self.CURRENT_STATE = self.S_ROBOT_ASSIST
      self.b_user_reached_timeout = False

      if not game.check_board():
        print(colored("BOARD HAS TO BE RESET", 'red'))

      while not game.check_board():
        print(game.current_board)
        game.current_board = game.get_board_event()
        #self.play_sound("something_went_wrong_trim.mp3", 1)
        # check if the user reached his max number of attempts

      if game.n_attempt_per_token >= game.n_max_attempt_per_token:
        print(colored("Max attempt reached", 'red'))
        self.S_ROBOT_MOVE_CORRECT_TOKEN = True
        self.b_user_reached_max_attempt = True
        #self.play_sound("max_attempt_trim.mp3", 3)
        self.agent_move_correct_token(game, agent)

    # get current move and check if it is the one expeceted in the solution list
    elif game.detected_token[0] == game.solution[game.n_correct_move] \
        and game.detected_token[2] == (game.solution.index(game.detected_token[0]) + 1):
      agent.action["congrats"].__call__(counter=game.n_attempt_per_token-1, facial_expression="happy", eyes_coords=(0, 0))
      game.outcome = 0
      print("correct_solution ", game.get_n_correct_move())
      self.CURRENT_STATE = self.S_ROBOT_ASSIST

    elif game.detected_token[0] != game.solution[game.n_correct_move] \
        or game.detected_token[2] != game.solution.index(game.detected_token[0]) + 1:
      game.outcome = 1
      print("wrong_solution")
      agent.action["compassion"].__call__(counter=game.n_attempt_per_token-1, facial_expression="sad", eyes_coords=(0,80))
      self.CURRENT_STATE = self.S_USER_MOVE_TOKEN_BACK
      #if agent replace with
      #self.agent_move_back(game, agent)
      self.user_move_back(game, agent)

      # check if the user reached his max number of attempts
      if game.n_attempt_per_token >= game.n_max_attempt_per_token:
        print(colored("Max attempt reached", 'red'))
        #self.play_sound("max_attempt_trim.mp3", 3)
        self.S_ROBOT_MOVE_CORRECT_TOKEN = True
        self.b_user_reached_max_attempt = True
        self.agent_move_correct_token(game, agent)

    agent.reset_facial_expression()
    self.b_agent_outcome_finished = True
    return self.b_agent_outcome_finished

  def agent_move_correct_token(self, game, agent):
    if agent.get_action_state() == 0:
      # game.react_time_per_token_spec_t0 = time.time()
      agent.cancel_action()
    print(colored("Robot or Therapist moves the correct token as the user reached the max number of attempts", "red"))
    # get the current solution
    token = game.get_token_sol()
    success = agent.action["max_attempt"].__call__(token=token, counter=game.n_attempt_per_token-1, facial_expression="neutral", eyes_coords=(0, 30))
    #success = agent.action["assistance"].__call__(lev_id=5, row=0, counter=game.n_attempt_per_token-1, token=token, facial_expression="neutral", eyes_coords=(0,60), tokens=[], delay_for_speech=1.0)
    while(game.detected_token != (token)):
     pass
    #agent.send_to_rest()
    print("Robot moved the token in the correct location")
    #input = raw_input("move the token in the correct position and press a button")
    self.CURRENT_STATE = self.S_ROBOT_ASSIST
    self.b_agent_moved_correct_token = True
    return self.b_agent_moved_correct_token

  def agent_move_back(self, game, agent):
    # user moved the token in an incorrect location
    # agent moved it back
    agent.cancel_action()
    token = game.detected_token
    success = agent.action["move_back"].__call__(who="agent", token=token, counter=game.n_attempt_per_token-1, facial_expression="neutral")
    while(game.detected_token != token):
      pass
    print("Robot moved back the token in its initial location")
    self.CURRENT_STATE = self.S_ROBOT_ASSIST
    self.b_agent_moved_token_back = True
    return self.b_agent_moved_token_back

  def user_move_back(self, game, agent):
    # user moved the token in an incorrect location
    # agent moved it back
    # get the initial location of the placed token and move back there
    token_id, token_from = game.get_token_init_loc(game.detected_token[0])
    success = agent.action["move_back"].__call__(who="user", token=game.get_token_sol(), counter=game.n_attempt_per_token-1, facial_expression="sad", eyes_coords=(0, 0))
    curr_token_id, _, curr_token_to = game.detected_token
    time_elapsed = 0
    current_time = time.time()
    while (curr_token_id == token_id and curr_token_to != token_from):
      curr_token_id, _, curr_token_to = game.detected_token
      time_elapsed = time.time() - current_time
      if time_elapsed >= 10.0:
        success = agent.action["move_back"].__call__(who="user", token=game.get_token_sol(),
                                                     counter=game.n_attempt_per_token - 1, facial_expression="neutral")

        current_time = time.time()

    print(colored("User moved back the token in its initial location", 'red'))
    self.CURRENT_STATE = self.S_ROBOT_ASSIST
    self.b_user_moved_token_back = True
    return self.b_user_moved_token_back

  def user_action(self, game, agent):
    '''user action state has been divided in three sub methods:
     1. user pick a token
     1.a check for timeout
        1.b timeout -> provide reengagement
        go back to 1
     2. agent provides feedback
     3. user places a token
     3.a user places a token back
     3.b user places a token in a goal location
      '''
    print("U_ACTION")

    def check_move_timeout(game, time_to_start):
      '''we need to check if the computed time (react+elapsed) is smaller than timeout
      if not, we need to trigger different actions:
      1. if the user has not picked a token (re-engage)
      2. if the user has picked a token, ask her to move it back
      '''
      current_time = time.time()
      elapsed_time = current_time - time_to_start
      if elapsed_time < game.timeout:
        return True
      else:
        return False

    def user_pick_token(sm, game, agent):
      game.react_time_per_token_spec_t0 = time.time()
      print("U_PICK")
      sm.CURRENT_STATE = sm.S_ROBOT_FEEDBACK
      detected_token, picked, _, _ = game.get_move_event()
      speech_ended = True
      agent.reset_speech_ended()
      # agent_offered_assistance = False
      while (not picked and (detected_token == [])):
        # if (time.time()-game.react_time_per_token_spec_t0)>5 and (time.time()-game.react_time_per_token_spec_t0)<10 and not agent_offered_assistance:
        #   sm.agent_provide_assistance(game, agent)
        #   agent_offered_assistance = True
        #check here if the speech has ended if so trigger the counter
        if check_move_timeout(game, game.react_time_per_token_spec_t0):
          if speech_ended and agent.has_speech_ended():
            game.react_time_per_token_spec_t0 = time.time()
            agent.reset_speech_ended()
            speech_ended = False
          detected_token, picked, _, _ = game.get_move_event()
        else:
          sm.b_user_reached_timeout = True
          game.react_time_per_token_spec_t1 = game.timeout
          game.elapsed_time_per_token_spec_t1 = game.timeout
          return False
      #here we check if the agent is still moving if so stop it
      if agent.get_action_state() == 0:
        #game.react_time_per_token_spec_t0 = time.time()
        agent.cancel_action()
        agent.send_to_rest()


      #TODO try to merge these two if conditions together
      game.elapsed_time_per_token_spec_t0 = time.time()
      game.react_time_per_token_spec_t1 = time.time() - game.react_time_per_token_spec_t0
      game.react_time_per_token_gen_t1 += game.react_time_per_token_spec_t1
      sm.b_user_picked_token = True
      return sm.b_user_picked_token



    ''' When the user picked the token we check where they place it'''
    '''either they can place it back '''

    def user_place(sm, game):
      '''user place has been divided into:
         1. user places a token back
         2. user places a token in a solution row
      '''
      print(colored("U_PLACE", 'green'))
      # check where the user will place the token
      def user_place_token_back(sm):
        print(colored("U_PLACE_BACK", 'green'))
        sm.CURRENT_STATE = sm.S_ROBOT_OUTCOME
        sm.b_user_placed_token_back = True
        game.elapsed_time_per_token_spec_t1 = time.time() - game.elapsed_time_per_token_spec_t0
        game.elapsed_time_per_token_gen_t1 += game.elapsed_time_per_token_spec_t1
        return sm.b_user_placed_token_back

      '''or they can place it in the solution row'''

      def user_place_token_sol(sm):
        print("U_PLACE_SOL")
        sm.CURRENT_STATE = sm.S_ROBOT_OUTCOME
        sm.b_user_placed_token_sol = True
        game.elapsed_time_per_token_spec_t1 = time.time() - game.elapsed_time_per_token_spec_t0
        game.elapsed_time_per_token_gen_t1 += game.elapsed_time_per_token_spec_t1
        return sm.b_user_placed_token_sol

      '''return where the user placed the token'''

      # here we check whether a token has been picked, and where it has been placed
      detected_token, picked, placed, moved_back = game.get_move_event()
      while (not placed):
        if check_move_timeout(game, game.elapsed_time_per_token_spec_t0):
          detected_token, _, placed, moved_back = game.get_move_event()
        else:
          sm.CURRENT_STATE = sm.S_ROBOT_OUTCOME
          break
      if (placed and moved_back):
        sm.CURRENT_STATE = sm.S_ROBOT_OUTCOME
        return user_place_token_back(sm)
      elif (placed and not moved_back):
        sm.CURRENT_STATE = sm.S_ROBOT_OUTCOME
        return user_place_token_sol(sm)
      else:
        sm.b_user_reached_timeout = True
        game.elapsed_time_per_token_spec_t1 = game.timeout
        game.react_time_per_token_spec_t1 = game.timeout
        return False

    self.CURRENT_STATE = self.S_USER_ACTION
    agent.reset_facial_expression()

    # if the user picks a token and SOCIABLE is active
    if user_pick_token(self, game, agent):
      if game.with_feedback:
        if self.agent_provide_feedback(game, agent):
          if user_place(self, game):
            return True
          else:
            print("Something went wrong with user place in SOCIABLE")
        else:
          print("Something went wrong with agent feedback")
      else:
        if user_place(self, game):
          return True
        else:
          print("Something went wrong with user place")
    else:
      print("user_pick_token==False-expect_timeout")
      self.CURRENT_STATE = self.S_ROBOT_OUTCOME

  def num_to_func_to_str(self, argument):
    switcher = {
      0: self.agent_provide_assistance,
      1: self.agent_reengage_user,
      2: self.agent_provide_feedback,
      3: self.user_action,
      4: self.agent_provide_outcome,
      5: self.agent_move_back,
      6: self.user_move_back,
      7: self.agent_move_correct_token
    }

    # get the function based on argument
    func = switcher.get(argument)

    # Execute the function
    return func()


def main():
  language = rospy.get_param("/language")
  config_path = rospy.get_param("/config_path")
  sentences_file = ""
  if language == "en_GB":
    sentences_file = config_path+"/sentences_"+language
  elif language == "es_ES":
    sentences_file =  config_path+"/sentences_"+language
  elif language == "ca_ES":
    sentences_file =  config_path+"/sentences_"+language
  user_id = rospy.get_param("/user_id")
  with_feedback = rospy.get_param("/with_feedback")
  objective = rospy.get_param("/objective")
  session_id = rospy.get_param("/session_id")
  timeout = rospy.get_param("/timeout")
  pro_user = rospy.get_param("/pro_user")


  bn_game_state = {'beg': 2, 'mid': 4, 'end': 5}
  bn_attempt = {'att_1': 0, 'att_2': 1, 'att_3': 2, 'att_4': 3}
  bn_agent_assistance = {'lev_0': 0, 'lev_1': 1, 'lev_2': 2, 'lev_3': 3, 'lev_4': 4, 'lev_5': 5}
  bn_user_react_time = {'fast': 5, 'normal': 10, 'slow': 15}
  bn_user_action = {'correct': 0, 'wrong': 1, 'timeout': 2}

  # we create the game instance
  game = Game(board_size=(5, 4), task_length=5, n_max_attempt_per_token=4,
              timeout = timeout, objective = objective, sociable = with_feedback,
              bn_game_state = bn_game_state,
              bn_attempt = bn_attempt,
              bn_agent_assistance = bn_agent_assistance,
              bn_user_react_time = bn_user_react_time,
              bn_user_action = bn_user_action)

  attempt = [i for i in range(1,len(bn_attempt) + 1)]
  # +1 (3,_,_) absorbing state
  game_state = [i for i in range(0, len(bn_game_state) + 1)]
  user_action = [i for i in range(0, len(bn_user_action))]
  state_space = (game_state, attempt, user_action)
  states_space_list = list(itertools.product(*state_space))
  state_space_index = [states_space_list.index(tuple(s)) for s in states_space_list]

  # we create the agent instance
  speech = Speech(language)
  face = Face()
  gesture = Gesture()
  path = os.path.abspath(__file__)
  dir_path = os.path.dirname(path)
  parent_dir_of_file = os.path.dirname(dir_path)

  policy_filename = parent_dir_of_file+"/robot-patient-interaction/"+str(user_id)+"/"+str(with_feedback)+"/"+str(session_id)+"/policy.pkl"
  tiago_agent = Robot(speech, sentences_file, policy_filename, face, gesture)
  path_name = parent_dir_of_file + "/robot-patient-interaction/" + str(user_id) + "/" + str(with_feedback) + "/" + str(session_id)
  tiago_agent.reset_facial_expression()
  tiago_agent.send_to_rest()

  if not os.path.exists(path_name):
    os.makedirs(path_name)
  else:
    print("Error the directory already exists")
    input = raw_input("Would you like to continue anyway?")

  file_params = path_name + "/log_params.csv"
  file_spec = path_name + "/log_spec.csv"
  file_gen = path_name + "/log_gen.csv"
  file_summary = path_name + "/log_summary.csv"
  file_bn_variables = path_name + "/bn_variables.csv"

  if pro_user == 0:
    pass
  elif pro_user == "s1":
    game.solution = ["698", "894", "947", "289", "693"]
  elif pro_user == "s2":
    game.solution = ["587", "667", "593", "619", "753"]
  elif pro_user == "s3":
    game.solution = ["499", "939", "857", "667", "584"]
  elif pro_user == "demo":
    game.solution = ["899", "929", "848", "694", "971"]


  print("You are playing with the following numbers: ", game.current_board)
  print("The solution of the game is ", game.solution)
  input = raw_input("Press a key to start:")
  if input == "q":
    exit(-1)

  entry_log_params = {"user_id":"user_id", "session":"session", "with_feedback":"with_feedback", "objective":"objective", "timeout":"timeout"}
  entry_log_spec = {'game_state': 'game_state', 'user_react_time': 'user_react_time', 'user_action': 'user_action',
                    'token_id': 'token_id', 'from': 'from', 'to': 'to',
                    'agent_assistance': 'agent_assistance', "react_time": 'react_time',
                    'elapsed_time': 'elapsed_time', "attempt": 'attempt', "timeout": 'timeout',
                    "agent_feedback": 'agent_feedback'}
  entry_log_gen = {"game_state":"game_state","token_id": 'token_id', 'user_action': 'user_action', "from": 'from', "to": 'to',
                   "avg_agent_assistance_per_move": 'avg_agent_assistance_per_move',
                   "cum_react_time": "cum_react_time", "cum_elapsed_time": "cum_elapsed_time",
                   "attempt": "attempt",
                   "timeout": "timeout", "n_agent_feedback": "n_agent_feedback"}
  entry_log_summary = {"n_attempt": "n_attempt", "n_timeout": "n_timeout", "n_agent_feedback": "n_agent_feedback",
                        "avg_lev_assistance": "avg_lev_assistance",
                          "tot_react_time": "tot_react_time",
                          "tot_elapsed_time": "tot_elapsed_time"}
  entry_bn_variables = {"game_state": "game_state", "attempt": "attempt",
                        "agent_assistance": "agent_assistance",
                        "user_action": "user_action"}

  log = Log(filename_params=file_params, fieldnames_params = entry_log_params,
            filename_spec=file_spec, fieldnames_spec=entry_log_spec, filename_gen=file_gen,
            fieldnames_gen = entry_log_gen, filename_sum = file_summary, fieldnames_sum = entry_log_summary,
            filename_bn_vars=file_bn_variables, fieldnames_bn_vars=entry_bn_variables)

  log.add_row_entry(log_filename=file_spec, fieldnames=entry_log_spec, data=entry_log_spec)
  log.add_row_entry(log_filename=file_gen, fieldnames=entry_log_gen, data=entry_log_gen)
  log.add_row_entry(log_filename=file_summary, fieldnames=entry_log_summary, data=entry_log_summary)
  log.add_row_entry(log_filename=file_bn_variables, fieldnames=entry_bn_variables, data=entry_bn_variables)
  log.add_row_entry(log_filename=file_params, fieldnames=entry_log_params, data=entry_log_params)


  data_log_params = {"user_id":user_id, "session":session_id, "with_feedback":with_feedback, "objective":objective, "timeout":timeout}
  log.add_row_entry(log_filename=file_params, fieldnames=entry_log_params, data=data_log_params)

  sm = StateMachine(1)

  #tiago_agent.action["instruction"].__call__("instruction_"+str(objective), facial_expression="neutral", eyes_coords=(0,0))

  while game.get_n_correct_move() < game.task_length:
    current_state=(game.map_game_state(), game.n_attempt_per_token, game.outcome)
    print(current_state)

    if sm.CURRENT_STATE.value == sm.S_ROBOT_ASSIST.value:
      #expected_token = game.get_token_sol()
      print("check if expected token has been moved")
      if game.check_unexpected_move():
        sm.CURRENT_STATE = sm.S_ROBOT_OUTCOME
      else:
        sm.CURRENT_STATE = sm.S_USER_ACTION
        sm.agent_provide_assistance(game, tiago_agent, state_index=states_space_list.index(tuple(current_state)), epsilon=0.3)
        game.avg_agent_assistance_per_move += game.agent_assistance

    elif sm.CURRENT_STATE.value == sm.S_USER_ACTION.value:
      print("Expected token ", game.solution[game.get_n_correct_move()])
      time_to_act = time.time()

      #game.with_feedback = 1#random.randint(0,1)
      sm.user_action(game, tiago_agent)
      game.total_elapsed_time += time.time() - time_to_act

    elif sm.CURRENT_STATE.value == sm.S_ROBOT_OUTCOME.value:
      # these are reported only because the variables are already reset when a correct move occurred
      sm.agent_provide_outcome(game, tiago_agent)

      if (game.outcome == 0 or (game.outcome == 2 and game.n_attempt_per_token == game.n_max_attempt_per_token)
          or (game.outcome == 1 and game.n_attempt_per_token == game.n_max_attempt_per_token)):
        data_log_gen = game.store_info_gen()
        log.add_row_entry(log_filename=file_gen, fieldnames=entry_log_gen, data=data_log_gen)
        game.reset_counters_gen()

      data_log_spec = game.store_info_spec(game.outcome)
      data_bn_variables = game.store_bn_variables(game.outcome)
      log.add_row_entry(log_filename=file_spec, fieldnames=entry_log_spec, data=data_log_spec)
      log.add_row_entry(log_filename=file_bn_variables, fieldnames=entry_bn_variables,
                        data=data_bn_variables)

      sm.update_counters(game)
      game.reset_counters_spec()
      game.reset_detected_token()

  tiago_agent.action["end_game"].__call__(facial_expression="happy")
  data_log_summary = game.store_info_summary()
  log.add_row_entry(log_filename=file_summary, fieldnames=entry_log_summary, data=data_log_summary)

  for instance_spec in game.move_info_spec_vect:
    print(instance_spec)

  for instance_gen in game.move_info_gen_vect:
    print(instance_gen)

  print(game.total_elapsed_time)


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
