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

	uint32_t McountL:8; /// Current milliseconds per encoder tick for mining head left motor (255==stopped)
	uint32_t McountR:8; /// Encoder tick count for mining head left motor

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

  unsigned char head_extend:7; // Extend mining head linear
  unsigned char padding_extend:1; 

	robot_power() { stop(); }
	void stop(void) {
		left=right=mine=dump=roll=head_extend=drive_stop; // all-stop
		high=dumpMode=mineMode=torqueControl=mineHooks=mineDump=mineEncoderReset=0;
	}
};


// ***** LEGACY STUFF FOR COMPATIBILITY ***** //
/**
  These are the field dimensions, in centimeters.
  Y runs up the field, from collection bin to mining area.  It's always positive.
  X runs across the field, from left to right side.  It's signed.
*/
enum {
	field_y_size=738, // Y-length of field, in centimeters
//	field_y_size=500, // Y-length of field, in centimeters for the test arena
	field_y_start_zone=183, // y end of start area, in centimeters
	field_y_mine_zone=field_y_start_zone+294, // y start of mining area
	field_y_xmit=-47, // y coordinate of IR transmitters behind bin
	field_x_xmit=70, // x coordinate between IR transmitters
//	field_y_mine_zone=field_y_start_zone+100, // y start of mining area for the test arena

	field_x_size=378, // X-width of field, in centimeters
	field_x_hsize=field_x_size/2,
	field_x_GUI=field_x_hsize+10, // start X for GUI display
	field_x_bin=156, // X-width of collection bin, in centimeters
	field_x_hbin=field_x_bin/2
};

enum {
	robot_rt_blinky=55, // robot centerline to blinky receiver center, right() direction
	robot_fw_blinky=-33 // robot centerline to blinky receiver center, forward() direction
};

enum {
/*
		DN 949 fully up
		DN 330 mining head will drag on bar
		DN 260 mining starts on level ground
		DN 240 conservative mining depth
		DN 180 fully down
*/
		head_mine_stop=250, // stop lowering at this mining height
		head_mine_start=310, // start mining at this height
		head_mine_drive=500, // normal driving height
		head_drive_safe=800, // can safely drive below this height (without tipping over)
		head_mine_dump=890, // dumping height
		head_mine_stow = 250, // stow height, where frame is level
		head_bar_clear=320, //mining head will not drag on bar
		// These 2 are used for speed control while aligning and releasing hooks
		count_stop = 1, // encoder count to stop on
		count_slow = 110, // run fast until you're here
	// These 2 are used to tell whether the box is at max or min height
	box_raise_max = 310,
	box_raise_min = 0
	};
/**
  This is a list of possible robot states.
  It's mostly maintained on the backend, but
  can be commanded from the front end.
*/
typedef enum {
	state_STOP=0, ///< EMERGENCY STOP (no motion)
	state_drive, ///< normal manual driving
	state_backend_driver, ///< drive from backend UI

	state_autonomy, ///< full autonomy start state
	state_raise, ///< raise conveyor before driving
	state_find_camera, ///< turn until camera is visible
	state_align_turnout, ///< autonomous: pivot to face start position
	state_align_drive, ///< autonomous: initial drive to start position
	state_align_turnin, ///< autonomous: turn to face lunabin
	state_align_back, ///< autonomous: drive back to contact lunabin

	state_drive_to_mine, ///< autonomous: drive to mining area

	/* Semiauto mine mode entry point: */
	state_mine_lower, ///< mining mode: lowering head, driving forward
	state_mine_stall, ///< mining mode: raising head (after stall)
	state_mine, // actually mine
	state_mine_raise, ///< existing mining mode: raise bucket

	state_drive_to_dump, ///< drive back to bin

	/* Semiauto dump mode entry point: */
	state_dump_contact, ///< final dock-and-dump mode: drive to contact bin
	state_dump_raise, ///< raising bucket
	state_dump_pull, ///< pull box up
	state_dump_rattle, ///< rattle mode to empty bucket
	state_dump_push, ///< push box back down
	state_dump_lower, ///< lowering bucket (after dump)

	/* Semiauto dump mode entry point: */
	state_stow, // begin stowing: raise bucket
	state_stow_clean, // clean bucket
	state_stowed, // finished stowing (wait forever)

	state_last ///< end state (repeat from mine_drive)
} robot_state_t;
const char *state_to_string(robot_state_t state);

/// This bitfield convey's the robot's software status.
class robot_status_bits {
public:
	unsigned char stop:1; ///< EMERGENCY STOP engaged
	unsigned char arduino:1; ///< arduino is connected correctly
	unsigned char located:1; ///< robot thinks it knows where it is
	unsigned char autonomy:1; ///< full-autonomy mode is engaged
	unsigned char semiauto:1; ///< semiauto mode is engaged
};

/** This class contains robot localization information. */
class robot_localization {
public:
// Raw camera-derived robot location (cm)
	float x,y,z;

// Robot's orientation relative to lunabin (degrees from lunabin)
//   angle==0 means facing directly away from lunabin (world +y = robot forward)
//   angle==90 means facing due right (world +x = robot forward)
	float angle;

// Mining head's vertical pitch, in degrees.
//    pitch==0 means the mining head is in rest configuration.
//    positive pitch pushes the mining head back up away from the ground
  float pitch;

// Confidence in our position (1.0: recent detection; 0.0: no idea)
	float confidence;


#ifdef __OSL_VEC2_H
// 2D vector utility functions:
#ifndef M_PI
#define M_PI 3.14159265358979323
#endif

	/*
	  Return a world coordinates unit 2D direction vector 
	  for this robot-relative angle.
	  Angle==0 is facing along the robot's forward axis, in the direction of motion.
	  Angle==90 is facing to the robot's right.
	*/
	vec2 dir_from_deg(float ang_deg=0.0) const {
		float ang=(this->angle+ang_deg)*M_PI/180.0;
		return vec2(sin(ang),cos(ang)); // x=sin, y=cos due to angle 0 being Y axis (weird!)
	}
	// Return a robot coordinates angle from this direction vector.
	//  angles range from -180 to +180
	float deg_from_dir(const vec2 &dir) const {
		float world_deg=(180.0/M_PI)*atan2(dir.x,dir.y); // x&y flipped, for same reason
		float deg=world_deg-this->angle;
		if (deg<-180.0) deg+=360.0;
		if (deg>+180.0) deg-=360.0;
		return deg; 
	}
	
	vec2 center(void) const { return vec2(this->x,this->y); }
	vec2 forward(void) const { return dir_from_deg(0.0); }
	vec2 right(void) const { return dir_from_deg(90.0); }
	
	/*
	  Convert robot body coordinates (right, forward) to world coordinates.
	*/
	vec2 world_from_robot(const vec2 &r) const {
		return center()+r.x*right()+r.y*forward();
	}

	

#endif
};


// ********** //


/**
 This class contains everything we currently know about the robot.
*/
class robot_base {
public:
robot_state_t state; ///< Current control state
	robot_status_bits status; ///< Current software status bits
	robot_sensors_arduino sensor;  ///< Current hardware sensor values
	robot_localization loc; ///< Location
	robot_power power; // Current drive commands

	bool autonomous;
};

#endif

