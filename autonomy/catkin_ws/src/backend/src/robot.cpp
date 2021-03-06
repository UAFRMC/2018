// robot.cpp

// Author: Ryker Dial
// Date Created: October 19, 2017
// Last Modified: October 30, 2017

#include <backend/robot.h>

Robot::Robot(ros::NodeHandle nh) : nh_(nh), MAX_VEL(1.0) {
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

	pkt_ = new A_packet_formatter<serial::Serial>(*serial_port_);

	// Configure Odometry
	double wheelbase = 1.3; // Separation of tracks in meters
	double meters_per_tick = 9*(19.0/27.0)*0.050/36.0; // meters of driving per wheel encoder tick, ==9 sprocket drive pegs at 50mm apart, 36 encoder counts per revolution
	odom_ = new Odom(wheelbase, meters_per_tick);

	// ***** Setup Publishers and Subscribers ***** //
	cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &Robot::cmdVelCallback, this);
	current_power_sub = nh_.subscribe("current_power",1,&Robot::currentPowerCallback,this);

	heartbeat_pub_ = nh_.advertise<std_msgs::UInt8>("heartbeat", 1);
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 30);
	// ********** //
}

Robot::~Robot() {
	delete serial_port_;
	delete pkt_;
	delete odom_;
}

void Robot::update() {
	// If there are bytes available on the serial port, read them in.
	try{
		if(serial_port_ -> available()) {
			A_packet p;
			while(-1 == pkt_ -> read_packet(p)) {}

			if (p.valid)
			{
				if (p.command==0)
				{
					ROS_INFO("Got echo packet back from robot");
				}
				else if (p.command==0xE)
				{
					ROS_INFO("Got ERROR (0xE) packet back from robot (length %d)", p.length);
				}
				else if (p.command==0x3)
				{
					ROS_INFO("Got Sensor Data");
					// sensor data
					if (!p.get(sensor))
					{
						ROS_INFO("Size mismatch on arduino -> PC sensor packet (expected %lu, got %d)",sizeof(sensor),p.length);
					}
					else // Process Sensor Data
					{
						heartbeat_msg_.data = sensor.heartbeat;
						heartbeat_pub_.publish(heartbeat_msg_);

						// For each side, choose the max of the encoder ticks
						uint8_t ticks_left = std::max(sensor.DL1count, sensor.DL2count);
						uint8_t ticks_right = std::max(sensor.DR1count, sensor.DR2count);
						odom_ -> updateOdom(ticks_left, ticks_right);
						odom_pub_.publish(odom_ -> odom_msg_);
					}
				}
				else
				{ // unknown packet type?!
					ROS_INFO("Got unknown packet type 0x%x length %d from robot",p.command,p.length);
				}
			}
		}
	}
	// We have lost connection to the Arduino, attempt to reconnect
	catch(serial::IOException & e) { 
		connectToSerial();
	}
}

// Transmit the current power packet
void Robot::sendPower() {
	ROS_INFO("Sending power: left:'%d',right:'%d",power.left,power.right);
	ROS_INFO("Sending power: mine:'%d',roll:'%d, dump:%d",power.mine,power.roll,power.dump);
	pkt_ -> write_packet(0x7,sizeof(power),&power);
}

// Attempts to connect to the Arduino over serial.
void Robot::connectToSerial() {
	while(ros::ok()) {
		try{
			// Find the Arduino Port
			do {
				serial_port_ -> setPort(find_device("Arduino"));
				if(serial_port_ -> getPort() == "") {
					ROS_INFO("Error: Could not find the Arduino. Retrying...");
				}
			} while(serial_port_ -> getPort() == "");

			// Connect to the Arduino
			ROS_INFO("Connecting to port '%s'", serial_port_ -> getPort().c_str());
			serial_port_ -> open();
			ROS_INFO("Connecting to port '%s' succeeded.", serial_port_ -> getPort().c_str());
			ros::Duration(1.0).sleep();
			break;
		}
		catch(serial::IOException & e) {
			ROS_INFO("Error Connecting to serial port. Retrying...");
		}
		catch(std::invalid_argument & e) {
			ROS_INFO("Error Connecting to serial port. Retrying...");
		}
	}	
}

int Robot::clamp(double value)
{
	if (value>127) value=127;
	else if(value<0) value=0;
	return int(value);

}

void Robot::cmdVelCallback(const geometry_msgs::TwistConstPtr & cmd_vel) {
	//ROS_INFO("Received command velocity");
	double wheelbase = odom_ -> getWheelbase();
	double left_vel = cmd_vel -> linear.x - wheelbase/2.0*(cmd_vel -> angular.z);
	double right_vel = cmd_vel -> linear.x + wheelbase/2.0*(cmd_vel -> angular.z);
	//ROS_INFO("left_vel: '%f',right_vel: '%f'",left_vel,right_vel);

	// Convert the left and right velocities to power commands
	double max_power_percentage = 0.3; // Never go above this percentage of full power
	power.left = clamp(max_power_percentage*(left_vel/MAX_VEL*63)+64);
	power.right = clamp(max_power_percentage*(right_vel/MAX_VEL*63)+64);

	if(left_vel==0 || right_vel ==0 )
	{
		power.left=power.right=64; //Stop when released
	}
	//ROS_INFO("left: '%d',right: '%d'",power.left,power.right);
}

void Robot::currentPowerCallback(const frontend::power &current_power)
{
	//ROS_INFO("Received Power from pilot");
	//ROS_INFO("mine:%f,dump:%f,roll:%f",current_power.mine,current_power.dump,current_power.roll);
	double max_mine_percentage=0.3;
	power.mine = clamp((max_mine_percentage*(current_power.mine*63))+64);
	power.dump = clamp((current_power.dump*63)+64);
	power.roll = clamp((current_power.roll*63) +64);
	//ROS_INFO("mine:%d,dump:%d,roll:%d",power.mine,power.dump,power.roll);
}