/**
 * @brief Listen to joy node, print them and dispatch them
 * @author Gabriel Urbain <gabriel.urbain@gmail.com>
 * @date 17/05/2015
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
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

ros::Publisher pub;

void chatterCallback(const sensor_msgs::Joy::ConstPtr j)
{
	geometry_msgs::Twist t;
	t.linear.x = j->axes.at(1);
	t.linear.y = 0;
	t.linear.z = 0;
	t.angular.x = 0;
	t.angular.y = 0;
	t.angular.z = j->axes.at(0);

	ROS_INFO("Linear speed: %f   \tand angular speed: %f", t.linear.x, t.angular.z);

	pub.publish(t);
}

int main(int argc, char **argv)
{
	// Init ROS node
	ros::init(argc, argv, "ir_joy");
	ros::NodeHandle n;
	
	// Init subscriber
	ros::Subscriber sub = n.subscribe("joy", 10, chatterCallback);

	// Init publisher
	pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	// Listening loop
	ros::spin();

	return 0;
}
