##
# Python script for fsm class unit tests
#
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

import rospy
import time
from fsm import *


# Try standalone version
fsm = BtomtFiniteStateMachine()
print(fsm.curr_state.__name__ + " " + fsm.prev_state.__name__)
fsm.start()
time.sleep(10)
print(fsm.curr_state.__name__ + " " + fsm.prev_state.__name__)
fsm.stop()
time.sleep(10)
print(fsm.curr_state.__name__ + " " + fsm.prev_state.__name__)

# Try ROS version
# rospy.init_node("ir_fsm")
# fsm = BtomtFiniteStateMachine()
# fsm.start_ros_node()