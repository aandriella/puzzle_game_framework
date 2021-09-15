#!/usr/bin/python
import os
#This is necessary to import code that is not rosbased in the main.py
path = os.path.abspath(__file__)
dir_path = os.path.dirname(path)
parent_dir_of_file = os.path.dirname(dir_path)
parent_parent_dir_of_file = os.path.dirname(parent_dir_of_file)


#ros import
import time
import rospy
from std_msgs.msg import String, Int8
import pygame


from iri_rfid_board_scanner.msg import BoardIds
from board_state.msg import TokenMsg
from board_state.msg import BoardMsg

class RfidBoard():
	def __init__(self):
		rospy.init_node('big_hero', anonymous=True)
		# subscriber for getting info from the board
		rospy.Subscriber("/rfid_board_scanner/board_ids", BoardIds, self.get_electro_board_callback)

		self.electro_board = list()
		self.electro_board_with_tokens = dict()
		self.current_board = dict()
		self.initial_board = dict()
		self.detected_token = ["",0,0]
		self.picked = False
		self.placed = False
		self.moved_back = False
		#getting the ids from the rfids sensors
		self.ids = list(map(int, rospy.get_param('/rfid_id').split()))
		self.ids = [ int(self.ids[i]) for i in range(len(self.ids))]

		#get the numbers for playing the game
		self.tokens = rospy.get_param('tokens_number')
		#coupling ids with numbers
		self.electro_board_from_ids_to_tokens = self.convert_electro_board_ids_to_tokens(self.ids, self.tokens)

	def play_sound(self, file_path, sleep):
		'''
		play a sound using pygame
		'''
		pygame.mixer.init()
		# pygame.mixer.music.load(parent_dir_of_file+'/sounds/wrong_move_trim.mp3')
		pygame.mixer.music.load(file_path)
		pygame.mixer.music.play(0)
		time.sleep(sleep)

	def get_tokens_id(self):
		'''
		get the tokens id from the board
		'''
		return self.tokens

	def get_initial_board(self):
		'''
		get initial board status
		'''
		return self.initial_board.copy()

	#get value from the topic
	def get_electro_board_ids(self):
		'''
		get the ids from the board, these ids need to be converted to the id on the top of the token
		'''
		return self.electro_board

	#topic subscriber
	def get_electro_board_callback(self, msg):
		'''callback from the topic subscriber'''
		self.electro_board = (msg.id)

	def get_electro_board(self):
		'''get the board with the correct ids on the top'''
		return self.get_tokens_by_ids(self.electro_board_from_ids_to_tokens, self.electro_board)

	def convert_electro_board_ids_to_tokens(self, ids, tokens):
		'''
		:param ids:
		:param tokens:
		:return: this method creates a dic where the ids are the keys and the values are the tokens
		'''
		from_ids_to_tokens = dict()
		for i in range(len(ids)):
			from_ids_to_tokens[ids[i]] = tokens[i]
		return from_ids_to_tokens

	def get_tokens_by_ids(self, from_ids_to_tokens_dict, msg):
		'''
		:param electro_board:
		:param msg:
		:return: given a dictionary where key is the id and value is the token change the msg according (0 empty !0 a token id)
		'''
		for i in range(len(msg)):
			if msg[i] == 0:
				# i+1 because the id start
				self.electro_board_with_tokens[i+1] = '0'
			else:
				self.electro_board_with_tokens[i+1] = from_ids_to_tokens_dict.get(msg[i])
		return self.electro_board_with_tokens

	def get_token_location_from_board_state(self, previous_board, token):
		'''
		:param: token
		:return: the location of the token given as input
		'''
		for key, val in previous_board.items():
			if val == token:
				return key

	def detect_grasped_token(self, previous_board, current_board):
		'''
		this function detects if a token has been picked
		Args:
		  previous_board: the status of the board at time t-1
		  current_board: the status of the board at time t
		'''

		picked_token = None
		for k in current_board.keys():
			if current_board.get(k) == '0' and previous_board.get(k) != current_board.get(k):
				# if the location now is empty check where it was before
				token_id = previous_board.get(k)
				location = k
				picked_token = (str(token_id), str(location))
		return picked_token

	def detect_placed_token(self, picked_token, current_board):
		'''
		this function detects when token has been placed on the board
		Args:
		  picked_token: the token the user has picked
		  current_board: the status of the board at time t
		'''
		token_id, loc_from = picked_token
		move_it_back = False
		placed_token = ()
		for key, val in current_board.items():
			if token_id == val and str(key) != loc_from:
				loc_to = key
				print("placed in a different location")
				placed_token = (str(token_id), str(loc_from), str(loc_to))
			elif token_id == val and str(key) == loc_from:
				loc_to = key
				print("placed in the same location")
				placed_token = (str(token_id), str(loc_from), str(loc_to))
				move_it_back = True
		return placed_token, move_it_back

	def detect_move(self, previous_board, current_board):
		'''
			this function detects if a token has moved from loc_from to loc_to
			Args:
			  previous_board: the status of the board at time t-1
			  current_board: the status of the board at time t
			'''

		move = ()
		new_location = 0
		for x1 in previous_board.keys():
			z = current_board.get(x1) == previous_board.get(x1)
			if not z:
				#print('location', x1)
				if (current_board.get(x1))!='0':
					token = (current_board.get(x1))
					#check where the token is in the new configuration
					prev_location = self.get_token_location_from_board_state(previous_board, token)
					for key, value in current_board.items():
						if token == value and prev_location !=key:
							new_location = key
					move = (token, prev_location, new_location)
		return move

	def get_current_board(self):
		'''get the current board status'''
		self.current_board = self.get_electro_board()
		return self.current_board.copy()

	def update_board(self, token_id, location):
		'''
		update the current board given a token and a location
		Args
		  token_id: the number
		  location: its location
		Return:
		  a board with the token token_id in location location
		'''
		if type(token_id)!=str:
			token_id = str(token_id)


		for loc, token in self.current_board.items():
			if token == (token_id):
				self.current_board[loc] = '0'

		for key in self.current_board.keys():
			if location == key:
				self.current_board[key] = token_id



# def wait_for_subscribers(publisher):
#     wait_rate = rospy.Rate(20)
#     while not publisher.get_num_connections():
#         wait_rate.sleep()


def main():
	rfid_board = RfidBoard()
	###############################################################
	rospy.init_node('big_hero', anonymous=True)
	pub_move = rospy.Publisher('detected_move', TokenMsg, queue_size=10)
	pub_board_status = rospy.Publisher('board_status', BoardMsg, queue_size=10)
	rate = rospy.Rate(10)  # 10hz
	#this is necessary in order to give the time to the board to start publishing
	rospy.sleep(2)

	initial_board = rfid_board.get_current_board()
	current_board = initial_board


	token_picked = False
	while not rospy.is_shutdown():

		detected_board = rfid_board.get_current_board()
		#check if a token has been picked
		picked_token = rfid_board.detect_grasped_token(current_board, detected_board)
		#if a token has been picked
		if (picked_token)!=None:

			# print("current ", current_board)
			# print("detected ", detected_board)
			print("grasped token:", picked_token)

			#rfid_board.play_sound(parent_dir_of_file + "/sounds/pull-out.ogg", 0.1)
			rate.sleep()
			rfid_board.detected_token = picked_token
			rfid_board.picked = True
			rfid_board.placed = False
			rfid_board.moved_back = False
			current_board = rfid_board.get_current_board()
			#print("token {}, from {}, to {}".format(detected_action[1], detected_action[2], detected_action[3]))
			print("***********************************")
			print("Picked token " + str(rfid_board.detected_token))
			print("Picked "+ str(rfid_board.picked))
			print("Placed " + str(rfid_board.placed))
			print("Moved back " + str(rfid_board.moved_back))
			print("***********************************")
			token_picked = True

		#check when a picked token is going to be placed by the user
		elif (token_picked):
			placed_token, moved_back = rfid_board.detect_placed_token(rfid_board.detected_token, detected_board)
			if len(placed_token)>0:
				#rfid_board.play_sound(parent_dir_of_file+"/sounds/pull-out.ogg", 0.1)
				rate.sleep()
				rfid_board.detected_token = placed_token
				rfid_board.picked = False
				rfid_board.placed = True
				rfid_board.moved_back = moved_back
				token_picked = False
				current_board = rfid_board.get_current_board()
				print("***********************************")
				print("Picked token " + str(rfid_board.detected_token))
				print("Picked " + str(rfid_board.picked))
				print("Placed " + str(rfid_board.placed))
				print("Moved back " + str(rfid_board.moved_back))
				print("***********************************")
		else:
			rfid_board.detected_token = []
			rfid_board.picked = False
			rfid_board.placed = False
			rfid_board.moved_back = False


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass







