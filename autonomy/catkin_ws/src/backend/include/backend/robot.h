// robot.h

// Author: Ryker Dial
// Date Created: October 19, 2017
// Last Modified: October 20, 2017

#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED

#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <vector>

// ***** Serial port helper function ***** //
// find_device()
// Given an identifier string, searches through an enumerated list of serial
//     ports for a matching device. Returns the first such device found, or
//     the null string if there are no matches.
static std::string find_device(std::string id) {
	// Search through available ports for desired device
	std::vector<serial::PortInfo> devices_found = serial::list_ports();
	std::vector<serial::PortInfo>::iterator iter;
	std::string port;
	for(iter = devices_found.begin(); iter != devices_found.end(); ++iter) {
		if( (iter -> description).find(id) != std::string::npos ) {
			ROS_INFO("Found device '%s' at port '%s'",
				(iter->description).c_str(), 
				(iter->port).c_str()
			);
			return (iter->port);
		}
	}
	return "";
}
// ********** //

// Begin class Robot
class Robot{
	public:
		Robot(ros::NodeHandle nh);
		void update();
	private:
		ros::NodeHandle nh_;
		serial::Serial* serial_port_;
		ros::Subscriber cmd_vel_sub_;

		void connectToSerial();

		void cmdVelCallback(const geometry_msgs::TwistConstPtr & cmd_vel);


}; // End class Robot

#endif // ROBOT_H_INCLUDED