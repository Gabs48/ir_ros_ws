/**
 * @brief Main loop to parse data from CHRobotics GP9 IMU and send it via a publisher.
 * @author Gabriel Urbain <gabriel.urbain@gmail.com>
 * @date 03/04/2015
 * @copyright 2015 iRobotique ASBL
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial.h"

#include <sstream>

using namespace std;


int main(int argc, char **argv)
{
	// Init ros and create a ros publisher
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Rate loop_rate(10);
	
	// Create and init serial listenner
	Imu imu;
	imu.init();

	// Main loop
	int count = 0;
	while (ros::ok()) {
//		std_msgs::String msg;

//		msg.data = "test";
//		ROS_INFO("%s", msg.data.c_str());
//		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
}
