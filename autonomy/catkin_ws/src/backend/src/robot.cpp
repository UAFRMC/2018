// robot.cpp

// Author: Ryker Dial
// Date Created: October 19, 2017
// Last Modified: October 20, 2017

#include <backend/robot.h>

Robot::Robot(ros::NodeHandle nh) : nh_(nh) {
	// Set serial configuration
	serial_config = {
		"",										// Port name. Found during enumeration
		57600,									// Baud rate
		serial::Timeout::simpleTimeout(1000),
		serial::eightbits,
		serial::parity_none,
		serial::stopbits_one,
		serial::flowcontrol_none
	};

	// Find Arduino Port
	while(serial_config.port == "") {
		serial_config.port = find_device("Arduino");
		if(serial_config.port == "") {
			ROS_INFO("Error: Could not find the Arduino. Retrying...");
		}
	}

	// Open the serial port
	ROS_INFO("Connecting to port '%s'", serial_config.port.c_str());
	serial::Serial sp(
		serial_config.port,
		serial_config.baudrate,
		serial_config.timeout,
		serial_config.bytesize,
		serial_config.parity,
		serial_config.stopbits,
		serial_config.flowcontrol
	);
	if(sp.isOpen()) {
		ROS_INFO("Connecting to port '%s' succeeded.", serial_config.port.c_str());
	}
	else {
		ROS_INFO("Error connecting to port.");
	}
}