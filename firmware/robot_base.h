/**
 Aurora Robotics general robot code.

 This is shared between the Arduino and the PC.

  Orion Sky Lawlor, lawlor@alaska.edu, 2014-03-23 (Public Domain)
*/
#ifndef __AURORA_ROBOTICS__ROBOT_H
#define __AURORA_ROBOTICS__ROBOT_H
#include <stdint.h> /* for uint32_t */

/** This is the Arduino's AREF analog reference voltage.
  It's the scale factor that gives true voltage output,
  and should be measured from the AREF pin against Arduino ground. */
#define AD_AREF_voltage (4.78)

/** This scale factor converts an
	Arduino Analog/Digital Data Number (0-1023)
	to a real voltage, assuming direct feed-in. */
#define AD_DN2low_voltage (AD_AREF_voltage/(1024.0))

/** This scale factor converts an
	Arduino Analog/Digital Data Number (0-1023)
	to a real voltage, after the resistor divider scaling.
*/
#define AD_DN2high_voltage ((AD_AREF_voltage)*(11.0)/1024.0)


/** This class contains all the robot's sensors, on Arduino, backend, or front end.
Raw sensor values go as bitfields, because many of them are 10-bit quantities:
	- Arduino A/D values are 10 bits each
	- Arena positions in cm are 9-10 bits each (arena is 378x738cm)
	- Blinky angle reports are about 9 bits each (500 samples per rotation)
*/
class robot_sensors_arduino
{
public:
	uint32_t battery:10; // raw A/D reading at top of battery stack (voltage = this*5*2000/384)
	uint32_t bucket:10; // raw A/D value from dump bucket lift encoder
	uint32_t latency:5; // Arduino control loop latency

	uint32_t Mstall:1;
	uint32_t DLstall:1;
	uint32_t DRstall:1;

	uint32_t stop:1; ///< EMERGENCY STOP button engaged
  	uint32_t heartbeat:3;

	uint32_t Mspeed:8; /// Current milliseconds per encoder tick for mining head left motor (255==stopped)
	uint32_t Mcount:8; /// Encoder tick count for mining head left motor

	uint32_t DL1count:8; /// Encoder tick count for front left drive wheel
	uint32_t DL2count:8; /// Encoder tick count for back left drive wheel
	uint32_t DR1count:8; /// Encoder tick count for right drive wheel
	uint32_t DR2count:8; /// Encoder tick count for back right drive wheel

	int32_t Rcount:16; /// Encoder tick for bag roll motor

  	uint32_t limit_top:8;
  	uint32_t limit_bottom:8;
  
  	uint32_t encoder_raw:16;
};

/**
 This class contains a power setting for each of the robot's actuators.

 The ":7" makes each a 7-bit field, with values:
	1 (reverse)
	64 (stop)
	127 (forward)
*/
class robot_power {
public:
	enum { drive_stop=64 };

	unsigned char left:7; // left drive wheels
	unsigned char high:1; // High power mode

	unsigned char right:7; // right drive wheels
	unsigned char torqueControl:1; // Drive backwards (for final dump)

	unsigned char mineHooks:1; //Line up with hooks
	unsigned char mineDump:1; // Run backwards and dump
	unsigned char mineEncoderReset:1; //Get ready to go out and mine again
	unsigned char motorControllerReset:1; //Reset BTS motor controller enable pin
	unsigned char padding:4; //Spare bits

	unsigned char mine:7; // mining head dig
	unsigned char mineMode:1; // if true, autonomously run mining head

	unsigned char dump:7; // storage bucket lift
	unsigned char dumpMode:1; // dock-and-dump mode

	unsigned char roll:7; //Roll bag
	unsigned char paddingRoll:1;


	robot_power() { stop(); }
	void stop(void) {
		left=right=mine=dump=roll=drive_stop; // all-stop
		high=dumpMode=mineMode=torqueControl=mineHooks=mineDump=mineEncoderReset=0;
	}
};

/**
 This class contains everything we currently know about the robot.
*/
class robot_base {
public:
	robot_sensors_arduino sensor;  ///< Current hardware sensor values
	robot_power power; // Current drive commands
};

#endif

