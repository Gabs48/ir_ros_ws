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
import threading


class FiniteStateMachine(threading.Thread):
	"The basic structure of a python Finite State Machine (FSM) using ROS. After \
	initialization,	the FSM can either be started with the run attribute or by \
	calling the *self.start_srv_name* service"


	def __init__(self):
		"Set ROS and FSM parameters"

		# Threading is used when launched independently from ROS
		# with the start command
		threading.Thread.__init__(self)

		# Finite State Machine parameters
		self.state_var = []
		self.curr_state = self.state_idle
		self.prev_state = self.state_idle
		self.running = False

		# ROS parameters
		self.queue_size = 1
		self.pub_rate = 0.5
		self.node_name = "ir_fsm"
		self.state_pub_name = "fsm_state"
		self.run_srv_name = "fsm_run"
		self.stop_srv_name = "fsm_stop"


	def run(self):
		"Run the FSM"

		# When start running	
		self.running = True	
		self.curr_state = self.state_init

		while self.running:
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

		return self.state_init


	def state_idle(self):
		"Initial state"

		return self.state_idle


	def __ros_state_pub(self):
		"Publish current state in ROS topic"

		rate = ros.Rate(self.pub_rate)
		while not ros.is_shutdown():
			ros.loginfo("Current State: " + str(self.curr_state.__name__))
			self.pub.publish(str(self.curr_state.__name__))
			rate.sleep()

		return


	def __ros_run_srv(self, msg):
		"Run the FSM via a ROS service"

		self.run()


	def __ros_stop_srv(self, msg):
		"Stop the FSM via a ROS service"

		self.stop()


	def start_ros_node(self):
		"Start ROS node and its publishers and services"

		ros.init_node(self.node_name)
		self.pub = ros.Publisher(self.state_pub_name, String, queue_size=self.queue_size)
		self.srv = ros.Service(self.run_srv_name, Empty, self.__ros_run_srv)
		self.srv = ros.Service(self.stop_srv_name, Empty, self.__ros_stop_srv)

		try:
			self.__ros_state_pub()
		except ros.ROSInterruptException:
			pass

		return


class BtomtFiniteStateMachine(FiniteStateMachine):

	def __init__(self):
		"Set ROS and FSM parameters"

		# Init parent class
		super().__init__()




if __name__ == '__main__':
	
	fsm = BtomtFiniteStateMachine()
	fsm.start_ros_node()