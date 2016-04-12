#!/usr/bin/env python

##
# Gathering of the different state machines used by MILO for its general behaviour 
# control.
# Gabriel Urbain <gabriel.urbain@gmail.com>
# Created in March 2016
# Copyright 2016 iRobotique ASBL
# 
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
##

import rospy as ros
from std_srvs.srv import *
from std_msgs.msg import String
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import Int32MultiArray
import threading
import demjson
from types import MethodType
import time
import subprocess


class FiniteStateMachine(threading.Thread):
	"The basic structure of a python Finite State Machine (FSM) using ROS. After \
	initialization,	the FSM can either be started with the run attribute or by \
	calling the *self.start_srv_name* service"


	def __init__(self,  filename_="default.json"):
		"Set ROS parameters and create FSM from file"

		# Threading is used when launched independently from ROS
		# with the start command
		threading.Thread.__init__(self)
		
		# Finite State Machine parameters
		self.state_var = []
		self.init_state = self.state_init
		self.curr_state = self.state_idle
		self.prev_state = self.state_idle
		self.running = False
		self.cust_var = dict()

		# Find a solution to create this automatically??
		# ROS nodes are always defined?
		self.bluetooth = [False, False, False, False, False, False, False, False]
		self.voice_sr = ""

		# ROS parameters
		self.queue_size = 1
		self.pub_rate = 0.5
		self.node_name = "ir_fsm"
		self.state_pub_name = "fsm_state"
		self.voice_pub_name = "voice_ss"
		self.motor_pub_name = "timed_move"
		self.bluetooth_sub_name = "listenbt"
		self.run_srv_name = "fsm_run"
		self.stop_srv_name = "fsm_stop"
		
		# Open json file
		self.filename = filename_
		with open(self.filename) as fsm_file:    
			data_str = fsm_file.read().replace('\n', '')
			data = demjson.decode(data_str)
			
			# Create methods from json file and add them to this class
			for elem in data:
				if "state_" in elem:   
					if data[elem]["type"] == "speak":
						self.__bind_state_speak(elem, data)
					elif data[elem]["type"] == "speak_store":
						self.__bind_state_speak_store(elem, data)
					elif data[elem]["type"] == "timed_move":
						self.__bind_state_timed_move(elem, data)
					elif data[elem]["type"] == "listen_store":
						self.__bind_state_listen_store(elem, data)
					elif data[elem]["type"] == "listen_branch":
						self.__bind_state_listen_branch(elem, data)
					elif data[elem]["type"] == "bluetooth_branch":
						self.__bind_state_bluetooth_branch(elem, data)
					elif data[elem]["type"] == "video":
						self.__bind_state_video(elem, data)
					elif data[elem]["type"] == "idle":
						self.__bind_state_idle(elem, data)
					else:
						self.__bind_state_idle(elem, data)
			
			# Refer init state
			for elem in data:
				if elem == "init_state":
					self.init_state = getattr(self, data[elem])


	def run(self):
		"Run the FSM"

		# When start running
		self.running = True
		self.curr_state = self.init_state

		while self.running:
			time.sleep(0.1)
			self.prev_state = self.curr_state
			self.curr_state = self.curr_state()

		# When stop running
		self.curr_state = self.state_idle
		return


	def stop(self):
		"Stop the FSM"

		self.running = False

		return


	def state_init(self):
		"Initial state"

		ros.logwarn("[Default Init] Verify that you refered the field 'init_state' in the json fsm file")

		return self.state_idle


	def state_idle(self):
		"Initial state"

		time.sleep(1)
		return self.state_idle


	def __bind_state_speak(self, name, data):
		"Binding function to create a state function of type SPEAK"
		
		classname = self
		
		# The function added to the class
		def func(self):

			ros.loginfo("[State Speak] " + str(name))
			ros.loginfo("Spoken: " + data[name]["content"]["content"].encode('utf-8'))
			classname.voice_pub.publish(data[name]["content"]["content"])
			
			return getattr(classname, data[name]["next"])
		
		func.__name__ = str(name)
		exec("self." + str(name) + " = MethodType(func, FiniteStateMachine, self)" ) \
			in globals(), locals()
		return


	def __bind_state_listen_branch(self, name, data):
		"Binding function to create a state function of type LISTEN BRANCH"
		
		classname = self
		
		# The function added to the class
		def func(self):

			ros.loginfo("[State Listen Branch] " + str(name))

			## Wait for the bluetooth activation

			## Start listening

			## Stop when it's down
			
			heard = raw_input("Entrez votre choix : ")
			for branch in data[name]["content"]["choices"]:
				if heard == str(branch):
					return getattr(classname, data[name]["content"]["choices"][branch])
				
			# If nothing is found, return the default choice
			return getattr(classname, data[name]["content"]["choices"]["default"])
		
		func.__name__ = str(name)
		exec("self." + str(name) + " = MethodType(func, FiniteStateMachine, self)" ) \
			in globals(), locals()
		return

	def __bind_state_bluetooth_branch(self, name, data):
		"Binding function to create a state function of type BLUETOOTH BRANCH"
		
		classname = self
		
		# The function added to the class
		def func(self):

			ros.loginfo("[State Bluetooth Branch] " + str(name))

			for switch in data[name]["content"]:
				if classname.bluetooth[int(switch)]:
					return getattr(classname, data[name]["content"][switch]['True'])
				else:
					return getattr(classname, data[name]["content"][switch]['False'])
				
			# If nothing is found, return the default choice
			return getattr(classname, data[name]["content"]["choices"]["default"])
		
		func.__name__ = str(name)
		exec("self." + str(name) + " = MethodType(func, FiniteStateMachine, self)" ) \
			in globals(), locals()
		return


	def __bind_state_speak_store(self, name, data):
		"Binding function to create a state function of type SPEAK STORE"

		classname = self
		
		# The function added to the class
		def func(self):

			ros.loginfo("[State Speak Store] " + str(name))
			## Here we speak the three parts of the sentence
			sentence = data[name]["content_1"]["content"].encode('utf-8') + \
				classname.cust_var[data[name]["content_2"]["variable"]] + \
				data[name]["content_3"]["content"].encode('utf-8')
			ros.loginfo("Spoken: " + sentence)
			classname.voice_pub.publish(sentence)
			
			return getattr(classname, data[name]["next"])
		
		func.__name__ = str(name)
		exec("self." + str(name) + " = MethodType(func, FiniteStateMachine, self)" ) \
			in globals(), locals()
		return


	def __bind_state_listen_store(self, name, data):
		"Binding function to create a state function of type LISTEN STORE"

		classname = self
		
		# The function added to the class
		def func(self):

			ros.loginfo("[State Listen Store] " + str(name))
			## Here we listen and store the value
			a = raw_input("Entrez votre choix : ")
			classname.cust_var[data[name]["content"]["variable"]] = a
			
			return getattr(classname, data[name]["next"])
		
		func.__name__ = str(name)
		exec("self." + str(name) + " = MethodType(func, FiniteStateMachine, self)" ) \
			in globals(), locals()
		return

	
	def __bind_state_timed_move(self, name, data):
		"Binding function to create a state function of type TIMED MOVE"
		
		classname = self
		
		# The function added to the class
		def func(self):

			ros.loginfo("[State Timed moved] " + str(name))
			## Here we move
			return getattr(classname, data[name]["next"])
		
		func.__name__ = str(name)
		exec("self." + str(name) + " = MethodType(func, FiniteStateMachine, self)" ) \
			in globals(), locals()
		return

	def __bind_state_video(self, name, data):
		"Binding function to create a state function of type VIDEO"
		
		classname = self
		
		# The function added to the class
		def func(self):

			ros.loginfo("[State Timed moved] " + str(name))
			
				## We project the video
			args = [data[name]["content"]["reader"]]

			# Add arguments to command line
			#args.extend(["-o", "local"])
			args.extend([data[name]["content"]["file"]])

			# Start batch process and quit
			ros.loginfo("[State Video] Subprocess: " + str(args))
			subprocess.call(args)

			return getattr(classname, data[name]["next"])
		
		func.__name__ = str(name)
		exec("self." + str(name) + " = MethodType(func, FiniteStateMachine, self)" ) \
			in globals(), locals()
		return
	
	def __bind_state_idle(self, name, data):
		"Binding function to create a state function of type IDLE"
		
		classname = self
		
		# The function added to the class
		def func(self):

			ros.loginfo("[State Idle] " + str(name))
			## Here we do nothing. Idle function
			return classname.state_idle()
		
		func.__name__ = str(name)
		exec("self." + str(name) + " = MethodType(func, FiniteStateMachine, self)" ) \
			in globals(), locals()
		return


	def __bind_state_init(self, name, data):
		"Binding function to create a state function of type INIT"
		
		classname = self
		
		# The function added to the class
		def func(self):

			ros.loginfo("[State Init] " + str(name))
			return getattr(classname, data[name]["next"])
		
		func.__name__ = str(name)
		exec("self." + str(name) + " = MethodType(func, FiniteStateMachine, self)" ) \
			in globals(), locals()
		return


	def __ros_state_pub(self):
		"Publish current state in ROS topic"

		rate = ros.Rate(self.pub_rate)
		while not ros.is_shutdown():
			ros.loginfo("Current State: " + str(self.curr_state.__name__))
			self.state_pub.publish(str(self.curr_state.__name__))
			rate.sleep()

		return


	def __ros_run_srv(self, msg):
		"Run the FSM via a ROS service"

		self.start()


	def __ros_stop_srv(self, msg):
		"Stop the FSM via a ROS service"

		self.stop()

	def __ros_bluetooth_sub(self, msg):
		"ROS subscriber to check the bluetooth"
		
		ros.loginfo("iciii" + str(msg))
		i = 0
		for val in msg.data:
			ros.loginfo("Value " + str(i) + ": " + str(val))
			if val == '1':
				self.bluetooth[i] = True
			else:
				self.bluetooth[i] = False
			i += 1

		ros.loginfo(self.bluetooth)

		return 


	def start_ros_node(self):
		"Start ROS node and its publishers and services"

		ros.init_node(self.node_name)
		self.state_pub = ros.Publisher(self.state_pub_name, String, queue_size=self.queue_size)
		self.voice_pub = ros.Publisher(self.voice_pub_name, String, queue_size=self.queue_size)
		self.motor_pub = ros.Publisher(self.motor_pub_name, ByteMultiArray, queue_size=self.queue_size)

		self.bluetooth_sub = ros.Subscriber(self.bluetooth_sub_name, Int32MultiArray, \
			callback=self.__ros_bluetooth_sub, queue_size=self.queue_size)

		self.run_srv = ros.Service(self.run_srv_name, Empty, self.__ros_run_srv)
		self.stop_srv = ros.Service(self.stop_srv_name, Empty, self.__ros_stop_srv)

		try:
			self.__ros_state_pub()
		except ros.ROSInterruptException:
			pass

		return


if __name__ == '__main__':
	
	fsm = FiniteStateMachine()
	fsm.start_ros_node()