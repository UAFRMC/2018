// robot.cpp

// Author: Ryker Dial
// Date Created: October 19, 2017
// Last Modified: October 20, 2017

#include <backend/robot.h>

Robot::Robot(ros::NodeHandle nh) : nh_(nh) {
	// ***** Setup the Serial Port ***** //
	// Set serial configuration
	serial_config_ = {
		"",										// Port name. Found during enumeration
		57600,									// Baud rate
		serial::Timeout::simpleTimeout(1000),
		serial::eightbits,
		serial::parity_none,
		serial::stopbits_one,
		serial::flowcontrol_none
	};

	// Find Arduino Port
	while(serial_config_.port == "") {
		serial_config_.port = find_device("Arduino");
		if(serial_config_.port == "") {
			ROS_INFO("Error: Could not find the Arduino. Retrying...");
		}
	}

	// Open the serial port
	ROS_INFO("Connecting to port '%s'", serial_config_.port.c_str());
	serial_port_ = new serial::Serial(
		serial_config_.port,
		serial_config_.baudrate,
		serial_config_.timeout,
		serial_config_.bytesize,
		serial_config_.parity,
		serial_config_.stopbits,
		serial_config_.flowcontrol
	);
	if(serial_port_ -> isOpen()) {
		ROS_INFO("Connecting to port '%s' succeeded.", serial_config_.port.c_str());
	}
	else {
		ROS_INFO("Error connecting to port.");
	}
	// ********** //

	// ***** Setup Publishers and Subscribers ***** //
	cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &Robot::cmdVelCallback, this);
	// ********** //
}

void::Robot::update() {
	// If there are bytes available on the serial port, read them in.
	if(serial_port_ -> available()) {
		size_t size = serial_port_ -> available();
		uint8_t* buffer = new uint8_t[size];
		size_t bytes_read = serial_port_ -> read(buffer, size);
		for(int i=0; i<bytes_read; ++i)
			printf("%02X ", buffer[i]);
		printf("\n");

		// Process the packet

		delete [] buffer;
	}
}

void Robot::cmdVelCallback(const geometry_msgs::TwistConstPtr & cmd_vel) {

}