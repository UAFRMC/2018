// backend_driver_node.cpp

// Author: Ryker Dial
// Date Created: October 19, 2017
// Last Modified: October 20, 2017

#include <ros/ros.h>
#include <backend/robot.h>

int main(int argc, char** argv) {
	// Setup the ROS node
	ros::init(argc, argv, "backend_driver_node");
	ros::NodeHandle nh;

	Robot robot(nh);

	ros::Time time_last_request = ros::Time::now();

	while(true) { // Main  program loop
		if(ros::Time::now().toSec() - time_last_request.toSec() >= 0.1) {
			robot.requestSensors();
			time_last_request = ros::Time::now();
		}

		robot.update();
		ros::spinOnce();
	}

	return 0;
}