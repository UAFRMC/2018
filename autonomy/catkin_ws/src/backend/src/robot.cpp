// robot.cpp

// Author: Ryker Dial
// Date Created: October 19, 2017
// Last Modified: October 20, 2017

#include <backend/robot.h>

Robot::Robot(ros::NodeHandle nh) : nh_(nh) {
	// ***** Setup the Serial Port ***** //
	// Set serial configuration
	serial_port_ = new serial::Serial(
		"",										// Port name. Found during enumeration
		57600,									// Baud rate
		serial::Timeout::simpleTimeout(1000),
		serial::eightbits,
		serial::parity_none,
		serial::stopbits_one,
		serial::flowcontrol_none
	);

	connectToSerial();
	// ********** //

	// ***** Setup Publishers and Subscribers ***** //
	cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &Robot::cmdVelCallback, this);
	// ********** //
}

void Robot::update() {
	// If there are bytes available on the serial port, read them in.
	try{
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
	// We have lost connection to the Arduino, attempt to reconnect
	catch(serial::IOException & e) { 
		while(true) {
			try{
				connectToSerial();
				break;
			}
			catch(serial::IOException & e) {
				ROS_INFO("Error Connecting to serial port.");
			}
		}
	}
}

void Robot::connectToSerial() {
	// Find Arduino Port
	do {
		serial_port_ -> setPort(find_device("Arduino"));
		if(serial_port_ -> getPort() == "") {
			ROS_INFO("Error: Could not find the Arduino. Retrying...");
		}
	} while(serial_port_ -> getPort() == "");

	// Open the serial port
	while(true) {
		try {
			ROS_INFO("Connecting to port '%s'", serial_port_ -> getPort().c_str());
			serial_port_ -> open();
			if(serial_port_ -> isOpen()) {
				break;
			}
		}
		catch(serial::IOException & e) {
			ROS_INFO("Error connecting to port '%s'. Retrying...", serial_port_ -> getPort().c_str());
		}
	}
	ROS_INFO("Connecting to port '%s' succeeded.", serial_port_ -> getPort().c_str());	
}

void Robot::cmdVelCallback(const geometry_msgs::TwistConstPtr & cmd_vel) {

}