// robot.h

// Date Created: October 19, 2017
// Last Modified: October 19, 2017

#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED

#include <ros/ros.h>

class Robot{
	public:
		Robot(ros::NodeHandle nh);
	private:
		ros::NodeHandle nh_;
		ros::Subscriber cmd_vel_sub_;
};

#endif // ROBOT_H_INCLUDED