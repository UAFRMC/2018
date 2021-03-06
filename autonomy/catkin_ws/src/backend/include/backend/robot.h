// robot.h

// Author: Ryker Dial
// Date Created: October 19, 2017
// Last Modified: October 22, 2017

#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED

#include <ros/ros.h>
#include <serial/serial.h>
#include <backend/serial_packet.h>
#include <backend/odom.h>
#include <frontend/power.h>
#include <robot_base.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <vector>
#include <algorithm>

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

// Begin class Robot
class Robot: public robot_base {
	public:
		Robot(ros::NodeHandle nh);
		~Robot();
		void update();
		void sendPower();
		int clamp (double);
	private:
		ros::NodeHandle nh_;
		serial::Serial* serial_port_;
		A_packet_formatter<serial::Serial>* pkt_;
		Odom* odom_;
		const double MAX_VEL;

		// Subscribers
		ros::Subscriber cmd_vel_sub_;
		ros::Subscriber current_power_sub;

		// Publishers
		ros::Publisher heartbeat_pub_;
		ros::Publisher odom_pub_;

		// Messages
		std_msgs::UInt8 heartbeat_msg_;

		void connectToSerial();

		void updateOdom(uint8_t ticks_left, uint8_t ticks_right);
		void cmdVelCallback(const geometry_msgs::TwistConstPtr & cmd_vel);
		void currentPowerCallback(const frontend::power &current_power);

}; // End class Robot

#endif // ROBOT_H_INCLUDED