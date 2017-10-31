// backend_driver_node.cpp

// Author: Ryker Dial
// Date Created: October 19, 2017
// Last Modified: October 30, 2017

#include <ros/ros.h>
#include <backend/robot.h>

int main(int argc, char** argv) {
	// Setup the ROS node
	ros::init(argc, argv, "backend_driver_node");
	ros::NodeHandle nh;

	Robot robot(nh);

	// Loop Timing Information
	const double POWER_COMMAND_RATE = 10; // Frequency to send power commands to the robot
	ros::Time time_last_request = ros::Time::now();

	while(ros::ok()) { // Main  program loop
		if(ros::Time::now().toSec() - time_last_request.toSec() >= 1.0/POWER_COMMAND_RATE) {
			robot.sendPower();
			time_last_request = ros::Time::now();
		}

		robot.update();
		ros::spinOnce();
	}

	return 0;
}