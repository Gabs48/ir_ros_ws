#!/usr/bin/env python

##
# Voice tools using Acapela for speech synthesis and Google for speech recognition
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

import speech_recognition as sr
import collections 
import rospy as ros
from std_srvs.srv import *
from std_msgs.msg import String
from std_msgs.msg import Bool
import subprocess

class Voice:

	NORM = "manon"
	HAPPY = "antoinehappy"
	SAD = "antoinesad"
	UPCLOSE = "antoineupclose"

	def __init__(self, n_saved_=10):
		"Create handles and variables for SR and SS"

		# Speech recognizer parameters
		last_recogn_list = []
		for i in range(n_saved_):
			last_recogn_list.append("void")
		self.last_recogn = collections.deque(last_recogn_list)
		self.r = sr.Recognizer()
		self.language = "fr-FR"
		self.sr_success_str = ""
		self.sr_success_bool = False
		self.mic_open = False

		# Sspeech synthetiser parameters
		self.ss_batch_name = "voice"

		# ROS parameters
		self.node_name = "ir_voice"
		self.pub_rate = 2
		self.pub_name = "voice_sr"
		self.srv_name = "rec_mic"
		self.ss_sub_name = "voice_ss"
		self.mic_sub_name = "mic_open"
		self.queue_size = 1


	def listen(self, source_wav_=None):
		"Speech recognition using Google SR API"

		# Open the microphone
		if source_wav_ != None:
			with sr.WavFile(source_wav_) as source:
				ros.loginfo("Listening to the WAV file: " + source_wav_)
				audio = self.r.record(source)
		else:
			ros.logwarn("Opening microphone")
			with sr.Microphone() as source:
				audio = self.r.listen(source)
			ros.logwarn("Closing microphone")

		# Recognize speech using Google Speech Recognition
		try:
			self.sr_success_str = self.r.recognize_google(audio, language=self.language)#, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
			self.sr_success_bool = True
			ros.loginfo("Google recognized: " + self.sr_success_str)

		except sr.UnknownValueError:
			self.sr_success_str = "NR"
			self.sr_success_bool = False
			ros.logerr("Google SR can not recognize audio")

		except sr.RequestError as e:
			err = "{0}".format(e)
			self.sr_success_str = "NA"
			self.sr_success_bool = False
			ros.logerr("Google SR API not accessible with code " + err)
		
		# Append answer on the left and remove the right one (FIFO)
		self.last_recogn.appendleft(self.sr_success_str)
		self.last_recogn.pop()

		return


	def speak(self, sentence_, voice_name_=NORM):
		"Speech synthesis using Acapela libraries, compiled in an example and wrapped in a \
		batch file whose name is defined in init. This file must be executable by the python \
		user for this class to work!"

		# Call batch subprocess
		args = [self.ss_batch_name]
		print(sentence_)

		# Add arguments to command line
		args.extend(["-t", voice_name_])
		args.extend([sentence_.decode('utf-8').encode("iso-8859-1","ignore")])

		# Start batch process and quit
		ros.loginfo("Acapela SS with " + voice_name_ + ": " + sentence_)
		subprocess.call(args)

		return


	def __ros_ss_sub(self, msg):
		"ROS subscriber to convert string to speech"
		
		self.speak(msg.data)

		return 


	def __ros_mic_sub(self, msg):
		"ROS subscriber to check if we can open the mic"
		
		self.mic_open = msg.data

		return 


	def __ros_pub(self):
		"ROS publisher of the last recognized sentence"

		rate = ros.Rate(self.pub_rate)
		while not ros.is_shutdown():
			ros.loginfo("Last recognized sentence: " + self.last_recogn[0])
			self.pub.publish(self.last_recogn[0])
			rate.sleep()

		return


	def __ros_srv(self, msg):
		"ROS service to enable microphone listening"

		self.listen("fr.wav")

		return TriggerResponse(self.sr_success_bool, self.sr_success_str)


	def start_ros_node(self):
		"This function allows to use the class as a ROS node with SS subscriber and SR publisher \
		instead of using it inside a python code"

		ros.init_node(self.node_name)
		self.pub = ros.Publisher(self.pub_name, String, queue_size=self.queue_size)
		self.sub = ros.Subscriber(self.ss_sub_name, String, callback=self.__ros_ss_sub, queue_size=self.queue_size)
		self.sub = ros.Subscriber(self.mic_sub_name, Bool, callback=self.__ros_mic_sub, queue_size=self.queue_size)
		self.srv = ros.Service(self.srv_name, Trigger, self.__ros_srv)

		try:
			self.__ros_pub()
		except ros.ROSInterruptException:
			pass

		return