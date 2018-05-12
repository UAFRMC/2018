/**
  Aurora Robotics Backend Code

  Orion Sky Lawlor, lawlor@alaska.edu, 2014-03-23 (Public Domain)
*/
#define AURORA_IS_BACKEND 1

#include <iostream>
#include <cmath>

#include "gridnav/gridnav_RMC.h"

#include "osl/quadric.h"
#include "../firmware/robot.h"
#include "aurora/robot.cpp"
#include "aurora/display.h"
#include "aurora/network.h"
#include "aurora/ui.h"
#include "aurora/robot_serial.h"

#include <SOIL/SOIL.h>

#include "ogl/event.cpp"
#include "osl/socket.cpp"

#include "osl/porthread.h" /* for threading */
#include "osl/porthread.cpp"

/** Video image analysis gets dumped here: */
#include "../aruco/viewer/location_binary.h"

// New vive localization
#include "osl/transform.h"
#include "osl/file_ipc.h"


#include "aurora/simulator.h"
#include <iostream>


using osl::quadric;

bool simulate_only=false; // -sim flag
bool nodrive=false; // -nodrive flag (for testing indoors)
bool big_field=false; // -big flag

/* Convert this unsigned char difference into a float difference */
float fix_wrap256(unsigned char diff) {
	if (diff>128) return diff-256;
	else return diff;
}


/**
  This class is used to localize the robot
*/
class robot_locator {
public:
	/** Merged location */
	robot_localization merged;

	/** Values from computer vision */
	location_binary vision;
	location_reader vision_reader;

	/** Update computer vision values */
	bool update_vision(const char *marker_path) {
		if (vision_reader.updated(marker_path,vision)) {
			if (vision.valid) {
				if (fabs(vision.x)<2.5 && vision.y>-0.5 && vision.y<3.0) {
					//merged=vision; // <- HACK!  Always trusts camera, even if it's bouncing around.
					merged.x=vision.x;
					merged.y=vision.y;
					merged.z=vision.z;
					merged.angle=vision.angle;
					merged.confidence=0.8; // vision.confidence;
					return true;
				}
				else {
					robotPrintln("Ignoring vision X %.1f  Y %.1f	angle %.0f\n",
						vision.x,vision.y,vision.angle);
				}
			}
			// merged.vidcap_count=vision.vidcap_count;
		}
		return false;
	}


	/* Update absolute robot position based on these incremental
	   wheel encoder distances.
	   These are normalized such that left and right are the
	   actual distances the wheels rolled, and wheelbase is the
	   effective distance between the wheels' center of traction.
	   Default units for vision_reader are in meters.
	*/
	void move_wheels(float left,float right,float wheelbase) {
	// Extract position and orientation from absolute location
		vec3 P=vec3(merged.x,merged.y,merged.z); // position of robot (center of wheels)
		double ang_rads=merged.angle*M_PI/180.0; // 2D rotation of robot

	// Reconstruct coordinate system and wheel locations
		vec3 FW=vec3(sin(ang_rads),cos(ang_rads),0.0); // forward vector
		vec3 UP=vec3(0,0,1); // up vector
		vec3 LR=FW.cross(UP); // left-to-right vector
		vec3 wheel[2];
		wheel[0]=P-0.5*wheelbase*LR;
		wheel[1]=P+0.5*wheelbase*LR;

	// Move wheels forward by specified amounts
		wheel[0]+=FW*left;
		wheel[1]+=FW*right;

	// Extract new robot position and orientation
		P=(wheel[0]+wheel[1])*0.5;
		LR=normalize(wheel[1]-wheel[0]);
		FW=UP.cross(LR);
		ang_rads=atan2(FW.x,FW.y);

	// Put back into merged absolute location
		merged.angle=180.0/M_PI*ang_rads;
		merged.x=P.x; merged.y=P.y; merged.z=P.z;
	}

};


/**
 This class represents everything the back end knows about the robot.
*/
class robot_manager_t
{
public:
	robot_current robot; // overall integrated current state

	robot_locator locator; // localization
	robot_telemetry telemetry; // next-sent telemetry value
	robot_command command; // last-received command
	robot_comms comms; // network link to front end
	robot_ui ui; // keyboard interface

	rmc_navigator navigator;

	robot_serial arduino;

	robot_simulator sim;

	robot_manager_t() {
		// HACK: zero out main structures.
		//  Can't do this to objects with internal parts, like comms or sim.
		memset(&robot,0,sizeof(robot));
		memset(&telemetry,0,sizeof(telemetry));
		memset(&command,0,sizeof(command));
		robot.sensor.limit_top=1;
		robot.sensor.limit_bottom=1;

		// Start simulation in random real start location
		sim.loc.y=90.0;
		sim.loc.x=((rand()%2)-0.5)*2.0*84.5;
		sim.loc.angle=(rand()%6)*60;
		sim.loc.confidence=1.0;

    // Add a few random obstacles to show off path planning
    for (int x=200;x<250;x++)
      navigator.mark_obstacle(x,300,30);
	}

	// Do robot work.
	void update(void);

private:
	// Autonomy support:
	double cur_time; // seconds since start of backend program
	double state_start_time; // cur_time when we entered the current state
	double mine_start_time; // cur_time when we last started mining
	double autonomy_start_time; // cur_time when we started full autonomy

	robot_state_t last_state;

	// Enter a new state (semi)autonomously
	void enter_state(robot_state_t new_state)
	{
		if (new_state==state_raise) { autonomy_start_time=cur_time; }
		// if(!(robot.autonomous)) { new_state=state_drive; }

		// Log state timings to dedicate state timing file:
		static FILE *timelog=fopen("timing.log","w");
		fprintf(timelog,"%4d spent %6.3f seconds in %s\n",
			(int)(cur_time-autonomy_start_time),
			cur_time-state_start_time, state_to_string(robot.state));
		fflush(timelog);

		// Make state transition
		last_state=robot.state; // stash old state
		robot.state=new_state;
		robotPrintln("Entering new state %s",state_to_string(robot.state));
		state_start_time=cur_time;
	}

	// Advance autonomous state machine
	void autonomous_state(void);

	// Raw robot.power levels for various speeds
	enum {
		power_full_fw=127, // forward
		power_stop=64,
		power_full_bw=1, // backward
	};

	// Dump bucket encoder target a/d values


	// Limit this value to lie in this +- range
	double limit(double v,double range) {
		if (v>range) return range;
		if (v<-range) return -range;
		else return v;
	}

	// Run autonomous mining, if possible
	bool tryMineMode(void) {
		if (robot.sensor.bucket<head_bar_clear) {
			robot.power.mine=100; // TUNE THIS!
			robot.power.mineMode = true; // Start PID based mining
			return true;
		}
		return false;
	}

	// Set the front wheels and bucket to natural driving posture
	//  Return true if we're safe to drive
	bool drive_posture() {
		int tolerance=10; // dead zone (to prevent hunting)
		if (robot.sensor.bucket<head_mine_drive-tolerance)
		{
			robot.power.dump=power_full_fw; // raise
			return false; // too low to drive yet
		}
		if (robot.sensor.bucket>head_mine_drive+tolerance)
		{
			robot.power.dump=power_full_bw; // lower
		}
		return true;
	}

	// Autonomous driving rate:
	//  Returns 0-1.0 float power value.
	float drive_speed(float forward,float turn=0.0) {
		return 0.8; // confident but conservative
	}

	// Autonomous feeler-based backing up: drive backward slowly until both switches engage.
	//  Return true when we're finally backed up properly.
	bool back_up()
	{
		if (sim.loc.y>55) { // keep driving toward the bin
			autonomous_drive(vec2(0.0,-1.0));
			return false;
		}
		else
			return true; // we're there!

		// const int back_slow=64-(drive_speed(-1.0)*64);
		// const int back_slow=64-(0.35*64); // faster back-up (hack!)
		if(!(drive_posture())) {return false;}
		else {
			// FIXME: back-up sensors?

			return true; // (robot.sensor.backL && robot.sensor.backR);
		}
	}

	// Autonomous driving: set powers to drive toward this field X,Y location
	//  Returns true once we're basically at the target location.
	bool autonomous_drive(vec2 target) {
		if (!drive_posture()) return false; // don't drive yet

		double drive_power=drive_speed(+1.0);
		vec2 cur(sim.loc.x,sim.loc.y); // robot location
		double angle=sim.loc.angle; // degrees (!?)
		double arad=angle*M_PI/180.0; // radians
		vec2 orient(sin(arad),cos(arad)); // orientation vector (forward vector of robot)
		vec2 should=normalize(cur-target); // we should be facing this way

		double turn=orient.x*should.y-orient.y*should.x; // cross product (sin of angle)
		double drive=dot(orient,should); // dot product (like distance)

		double t=limit(turn*0.5,drive_power);
		double d=limit(drive*0.3,drive_power);
		double L=-d-t;
		double R=-d+t;
		robot.power.left=64+63*limit(L,0.5);
		robot.power.right=64+63*limit(R,0.5);

		return length(cur-target)<20.0; // we're basically there
	}

	// Force this angle (or angle difference) to be between -180 and +180,
	//   by adding or subtracting 360 degrees.
	void reduce_angle(double &angle) {
		while (angle>=180) angle-=360; // reduce
		while (angle<-180) angle+=360; // reduce
	}

	// Autonomous turning: rotate robot so it's facing this direction.
	//  Returns true once we're basically at the target angle.
	bool autonomous_turn(double angle_target_deg=0.0,bool do_posture=true)
	{
		if (do_posture) { if (!drive_posture()) return false; } // don't drive yet
		double angle_err_deg=sim.loc.angle-angle_target_deg;
		reduce_angle(angle_err_deg);
		robotPrintln("Autonomous turn to %.0f from %.0f deg\n",
			angle_target_deg, sim.loc.angle);

		double turn=angle_err_deg*0.1; // proportional control
		double maxturn=drive_speed(0.0,1.0);
		turn=limit(turn,maxturn);
		robot.power.left=64-63*turn;
		robot.power.right=64+63*turn;
		return fabs(angle_err_deg)<5.0; // angle error tolerance
	}

	// Make sure we're still facing the collection bin.  If not, pivot to face it.
	bool check_angle() {
		if (robot.loc.confidence<0.2) return true; // we don't know where we are--just keep driving?
		double target=180.0/M_PI*atan2(robot.loc.x,robot.loc.y+200.0);
		double err=sim.loc.angle-target;
		robotPrintln("check_angle: cur %.1f deg, target %.1f deg",sim.loc.angle,target);
		reduce_angle(err);
		if (fabs(err)<10.0) return true; // keep driving--straight enough
		else return autonomous_turn(target,false); // turn to face target
	}
};

/** X,Y field target location where we drive to, before finally backing up */
vec2 dump_drive_loc(0,100);

// Return true if the mining head is stalled (according to our sensors
bool is_stalled(const robot_current &robot) {
	return robot.sensor.Mstall;
}


/* Utility function: slow down speed as cur approaches target
  Returns false if already past target.
*/
bool speed_limit(int &howfast,int cur,int target,int dir=+1)
{
	int dist_left=(target-cur)*dir;
	if (dist_left<0) {
		return false;
	}
	int max_speed=10+dist_left/5;
	if (howfast>max_speed) howfast=max_speed;
	return true;
}


void robot_manager_t::autonomous_state()
{
	robot.power.stop(); // each state starts from scratch

// Drive constants
	float power_drive_float=drive_speed(1.0); // autonomous drive speed: 0.2 for concrete, 0.3 for dust (careful), 0.35 for fast autonomy
	int power_drive_fw=(int)(64+power_drive_float*63); // driving speed
	int power_stuck_fw=(int)(64+0.45*63); // un-stuck speed
	int power_drive_bw=(int)(64-power_drive_float*63);

	//float robot_distance=sqrt(robot.loc.x*robot.loc.x+robot.loc.y*robot.loc.y);
	float robot_distance=robot.loc.y;

	double time_in_state=cur_time-state_start_time;
	robotPrintln("In state %s for %.1f seconds...\n", state_to_string(robot.state), time_in_state);

	// full autonomy start
	if (robot.state==state_autonomy) {
		robot.autonomous=true;
		enter_state(state_raise);
	}
	// raise: raise the mining head to clear ground for driving
	else if (robot.state==state_raise)
	{
		if(robot.sensor.bucket<head_mine_drive && time_in_state<5.0)// raises until bucket_drive
		{
			robot.power.dump=power_full_fw; // raise bin
		}
		else{
			enter_state(state_find_camera);
		}
	}
	//state_find_camera: line up with centerline
	else if (robot.state==state_find_camera)
	{
		if (!drive_posture()) { /* correct posture first */ }
		else if (robot.loc.confidence>0.5) { // we know where we are!
			sim.loc=robot.loc; // reset simulator to real detected values

			enter_state(state_align_turnout);
		}
		else // don't know where we are yet--turn left
		{
			bool stop=false;
			if (time_in_state>20.0 && fmod(time_in_state,10.0)<1.5)
			{ // stop every 10 seconds (to avoid camera blur)
				stop=true;
			}

			if (!stop) {
				if (time_in_state<40.0) { // turn left
					robot.power.left=power_drive_bw;
					robot.power.right=power_drive_fw;
				}
				else if (time_in_state<80.0) { // stuck??? maybe turn right?
					robot.power.left=power_drive_fw;
					robot.power.right=power_drive_bw;
				}
				else { // HELP!
					enter_state(state_drive);
				}
			}
		}
	}

	///< autonomous: rotate to face turning spot
	else if (robot.state==state_align_turnout) {
		double target_angle=-80.0; // degrees from Lunabin center
		if (sim.loc.x>0) target_angle=-target_angle; // right side? turn left.
		if (autonomous_turn(target_angle) || time_in_state>20.0) {
			enter_state(state_align_drive);
		}
	}

	///< autonomous: initial drive to line up with centerline
	else if (robot.state==state_align_drive) {
		bool x_aligned=fabs(sim.loc.x)<20.0; // on centerline
		if (!drive_posture()) { /* correct posture first */ }
		else if (x_aligned || time_in_state>20.0) { // done!
			enter_state(state_align_turnin);
		}
		else { // drive backward to turn area (stop if we hit wall)
			vec2 target(100.0,100.0);
			if (sim.loc.x>0.0) target.x*=-1.0; // flip target around
			if (autonomous_drive(target)) enter_state(state_drive_to_mine);
		}
	}

	///< autonomous: rotate to face directly toward bin center
	else if (robot.state==state_align_turnin) {
		double target=0.0; // 180.0/M_PI*atan2(sim.loc.x,sim.loc.y);
		if (autonomous_turn(target) || time_in_state>20.0) {
			enter_state(state_drive_to_mine);
		}
	}

	///< autonomous: drive backwards to contact lunabin for initial orientation
	///  DISABLED STATE in 2015 version
	else if (robot.state==state_align_back) {
		if (back_up() || time_in_state>30.0)
		{
			enter_state(state_drive_to_mine);
		}
	}

	//state_drive_to_mine: Drive to mining area
	//TODO:Currently proportional drive. We can go high power to avoid obstacles
	else if (robot.state==state_drive_to_mine)
	{
		if (drive_posture()) {
			
			double target_Y=field_y_mine_zone+20; // mining area distance (plus buffer)
			double err_Y=target_Y-robot_distance;
			robot.power.left=power_drive_fw;
			robot.power.right=power_drive_fw;
			if (err_Y<0.0)  // we're there now
			{
				enter_state(state_mine_lower); // start mining!
			}
			check_angle();

			if (time_in_state>20.0) { // stuck?  high power mode!
				robot.power.left=robot.power.right=power_stuck_fw;
			}
		}
	}

	//Enter Semiauto mine modes

	//state_mine_lower: enter mining state
	else if (robot.state==state_mine_lower) {
		robot.power.dump=power_full_bw; // lower bucket
		tryMineMode();

		if(robot.sensor.bucket<=head_mine_start || time_in_state>10.0)
		{
			mine_start_time=cur_time; // update mine start time
			enter_state(state_mine);
		}
	}
	else if (robot.state==state_mine)
	{
		if (!tryMineMode()) { // too high to mine (sanity check)
			robot.power.dump=power_full_bw; // lower bucket
		} else {
		if (robot.sensor.bucket>head_mine_stop) {
			robot.power.dump=64-20; // lower gently
		}

		double mine_time=cur_time-mine_start_time;
		double mine_duration=12.0;
		if(	big_field || (
			robot_distance<field_y_size-50 && // field left to mine
			mine_time<mine_duration)) // and there's room in the bin
		{ // keep mining

			if (robot.sensor.bucket<head_mine_start && (fmod(mine_time,2.0)<0.3)) {
			// ready to mine: slowly creep forward (with PID)
				robot.power.left=64+35;
				robot.power.right=64+35;
			}

			if(robot.sensor.Mstall) { enter_state(state_mine_stall);}
		}
		else { enter_state(state_mine_raise);} // done mining
		}
	}

	// state_mine_stall: Detect mining head stall. Raise head until cleared
	else if (robot.state==state_mine_stall)
	{
		tryMineMode(); // Start PID based mining
		if(robot.sensor.Mstall && robot.sensor.bucket<head_mine_start)
		{
			robot.power.dump=power_full_fw; // raise bucket
		}
		else {enter_state(state_mine);} // not stalled? Then back to mining
	}

	//state_mine_raise: Raise mining conveyor before starting to backup towards Lunarbin
	else if (robot.state==state_mine_raise)
	{
		if(robot.sensor.bucket<head_mine_drive && time_in_state<10.0)
		{
			robot.power.dump=power_full_fw;
		}
		else
		{
			if (big_field) enter_state(state_drive);
			else enter_state(state_drive_to_dump);
		}
	}

	// Drive back to bin
	else if (robot.state==state_drive_to_dump)
	{
		autonomous_drive(dump_drive_loc);

		if (sim.loc.y<150) { enter_state(state_dump_contact);}
	}


	//Semiauto dump mode entry point: dock and dump mode
	else if (robot.state==state_dump_contact) // final backup to Lunarbin
	{
		if (back_up() || time_in_state>30.0)
		{
		  enter_state(state_dump_raise);
		}
	}

	// raise bucket to dump
	else if (robot.state==state_dump_raise)
	{
		// FIXME: Time this process
		if(robot.sensor.bucket<head_mine_dump && time_in_state<10.0)
		{
			robot.power.dump=power_full_fw;
	  }
		else
		{
		  enter_state(state_dump_pull);
		}
	}
	else if(robot.state==state_dump_pull)
	{
		int howfast=32;
		int cur=(signed short)robot.sensor.Rcount;
		int target=box_raise_max;
		if (!speed_limit(howfast,cur,target,+1)  || time_in_state>15.0)
			enter_state(state_dump_rattle);
		else
			robot.power.roll=64+howfast; // forward
	}
	// Give dust time to flow out (maybe gentle rattle?)
	else if (robot.state==state_dump_rattle)
	{
		robot.power.dump=(fmod(time_in_state,0.2)>0.1)?power_full_fw:power_full_bw; // empty out conveyor (and rattle)
		if(time_in_state>2.0) {
			enter_state(state_dump_push);
		}
	}
	//Push fabric back after dumping
	else if(robot.state==state_dump_push)
	{
		int howfast=32;
		int cur=(signed short)robot.sensor.Rcount;
		int target=0;
		if (!speed_limit(howfast,cur,target,-1)  || time_in_state>15.0)
			enter_state(state_dump_lower);
		else
			robot.power.roll=64-howfast; // backward
	}
	// lower bucket to safe driving height
	else if (robot.state==state_dump_lower)
	{
		if(robot.sensor.bucket>head_drive_safe  && time_in_state<20.0)
		{
			robot.power.dump=power_full_bw;
		}
		else
		{
			if (big_field) enter_state(state_drive); // manual
			else enter_state(state_drive_to_mine);
		} // back to start again

	}
	else if (robot.state==state_stow)
	{
		if (robot.sensor.bucket>head_mine_stow) {
			robot.power.mine=power_full_fw; // empty out conveyor (and rattle)
		}
		if (robot.sensor.bucket>head_mine_dump || time_in_state>40.0)
		{
			enter_state(state_stow_clean);
		}
	}
	else if (robot.state==state_stow_clean)
	{
		if (robot.sensor.bucket>head_mine_stow) {
			robot.power.dump=power_full_bw; // keep lowering
		}
	}
	else if (robot.state==state_stowed)
	{
		/* wait here forever */
	}
	else
	{ // what?  unrecognized state?!  manual mode...
		robotPrintln("Autonomy: unrecognized state %s (%d)!?\n",state_to_string(robot.state), robot.state);
		enter_state(state_drive);
	}

	if (nodrive)
	{ // do not drive!  (except for state_drive)
		robotPrintln("NODRIVE");
		robot.power.left=robot.power.right=64;
	}
}




robot_manager_t *robot_manager;

unsigned int video_texture_ID=0;

void robot_manager_t::update(void) {
	cur_time=0.001*glutGet(GLUT_ELAPSED_TIME);

#if 1 /* enable for backend UI: dangerous, but useful for autonomy testing w/o frontend */
	// Keyboard control
	ui.update(oglKeyMap,robot);

	// Click to set state:
	if (robotState_requested<state_last) {
		robot.state=robotState_requested;
		robotPrintln("Entering new state %s (%d) by backend UI request",
			state_to_string(robot.state),robot.state);
		robotState_requested=state_last; // clear UI request
	}
#endif

  // Check for an updated location from the vive
  
	static osl::transform robot_tf;
	static file_ipc_link<osl::transform> robot_tf_link("robot.tf");
	if (robot_tf_link.subscribe(robot_tf)) {
	  robot.loc.x=robot_tf.origin.x-field_x_hsize; // make bin the origin
	  robot.loc.y=robot_tf.origin.y;
	  robot.loc.z=robot_tf.origin.z;
	  
	  robot.loc.angle=(180.0/M_PI)*atan2(robot_tf.basis.x.x,robot_tf.basis.x.y);
	  robot.loc.pitch=(180.0/M_PI)*robot_tf.basis.x.z;
	}
  


/*
// Check for an updated location from the "camera" vision application
	if (locator.update_vision("../aruco/viewer/marker.bin")) {
  	robot.loc=locator.merged; // copy out robot location

//		robotPrintln("Location: X %.1f   Y %.1f   angle %.0f  (%s)",
//			bin.x,bin.y,bin.angle,bin.valid?"valid":"invalid");

		if (true)
		{
			robot_localization &bin=locator.merged;

			float marker_dx=-60.0; // centimeters from bin center (X==0) to marker center (was -50 for marker to one side)
			float marker_dy=0.0; // centimeters from bin lip (Y==0) to marker
			vec2 loc(100.0*bin.x+marker_dx,100.0*bin.y+marker_dy); // center of camera

			float arad=bin.angle*M_PI/180.0; // radians
			vec2 right(cos(arad),-sin(arad)); // robot's right-hand axis
			float camera_right=+67.0; // centimeters from camera to robot center
			loc+=camera_right*right; // shift from camera to robot center

			robot.loc.x=loc.x;
			robot.loc.y=loc.y;
			robot.loc.z=100.0*bin.z;
			robot.loc.angle=bin.angle;
			robot.loc.confidence=std::max(robot.loc.confidence+0.2,0.0);
		}


		static uint32_t last_vidcap=-1;
		if (bin.vidcap_count!=last_vidcap) {
			robotPrintln("Reading updated vidcap texture");
			video_texture_ID=SOIL_load_OGL_texture(
				"../aruco/viewer/vidcap.jpg",
				0,video_texture_ID,
				SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y);
			last_vidcap=bin.vidcap_count;
		}
	//}*/

// Check for a command broadcast (briefly)
	int n;
	while (0!=(n=comms.available(10))) {
		if (n==sizeof(command)) {
			comms.receive(command);
			if (command.command==robot_command::command_STOP)
			{ // ESTOP command
				enter_state(state_STOP);
				robot.power.stop();
				robotPrintln("Incoming STOP command");
			}
			else if (command.command==robot_command::command_state)
			{
				if (command.state>=state_STOP && command.state<state_last)
				{
					robot.state=(robot_state_t)command.state;
					telemetry.ack_state=robot.state;
					robotPrintln("Entering new state %s (%d) by frontend request",
						state_to_string(robot.state),robot.state);
				} else {
					robotPrintln("ERROR!  IGNORING INVALID STATE %d!!\n",command.state);
				}
			}
			else if (command.command==robot_command::command_power)
			{ // manual driving power command
				robotPrintln("Incoming power command: %d bytes",n);
				if (robot.state==state_drive)
				{
					robot.autonomous=false;
					robot.power=command.power;
				}
				else
				{
					robotPrintln("IGNORING POWER: not in drive state\n");
				}
			}
		} else {
			robotPrintln("ERROR: COMMAND VERSION MISMATCH!  Expected %d, got %d",
				sizeof(command),n);
		}
	}

// Perform action based on state recieved from FrontEnd
	//E-Stop command
	if(robot.state==state_STOP)
	{// All stop
		robot.power.stop();
		state_start_time=cur_time;
	}
	else if (robot.state==state_drive)
	{ // do nothing-- already got power command
		state_start_time=cur_time;
	}
	else if (robot.state==state_backend_driver)
	{ // set robot power from backend UI
		robot.power=ui.power;
		printf("Backend driver dump: %d\n",robot.power.dump);
	}
	else if (robot.state>=state_autonomy) { // autonomous mode!
		autonomous_state();
	}

	//Variables to determine if you can raise or lower storage thingy
	bool can_raise_up=true;
	bool can_raise_down=true;

	//Detect soft encoder limiters
	if(robot.sensor.Rcount>=box_raise_max)
		can_raise_up=false;
	if(robot.sensor.Rcount<=0)
		can_raise_down=false;

	//Detect limit switches and reset encoder offset if needed
	if(robot.sensor.limit_top%2==0)
		can_raise_up=false;
	if(robot.sensor.limit_bottom%2==0)
		can_raise_down=false;

	//Stop raise/lower if limit detected
	if (!robot.power.torqueControl) //Override limit switches in torque control
	{
		if(robot.power.roll>64&&!can_raise_up)
			robot.power.roll=64;
		if(robot.power.roll<64&&!can_raise_down)
			robot.power.roll=64;
	}

	// Send commands to Arduino
	if (simulate_only) { // build fake arduino data
		robot.status.arduino=1; // pretend it's connected
		robot.sensor.bucket=sim.bucket*(950-179)+179;
		robot.sensor.Mcount=0xff&(int)sim.Mcount;
		robot.sensor.Rcount=0xffff&(int)sim.Rcount;
	} else { // real arduino
		robot_sensors_arduino old_sensor=robot.sensor;
		arduino.update(robot);
		
		// No hardware bucket height sensor: simulate in software
		robot.sensor.bucket=sim.bucket*(950-179)+179;

		//Reset encoder offset if needed
		if(robot.sensor.limit_top%2==0)
		{
			arduino.Rdiff+=box_raise_max-robot.sensor.Rcount;
			robot.sensor.Rcount=box_raise_max;
		}
		if(robot.sensor.limit_bottom%2==0)
		{
			arduino.Rdiff-=robot.sensor.Rcount;
			robot.sensor.Rcount=0;
		}

		float wheelbase=130.0; // cm between wheels (tracks)

		float drivecount2cm=9*(19.0/27.0)*0.050/36*100.0; // cm of driving per wheel encoder tick, ==9 sprocket drive pegs at 50mm apart, 36 encoder counts per revolution

		locator.move_wheels(
			fix_wrap256(robot.sensor.DL1count-old_sensor.DL1count)*drivecount2cm,
			fix_wrap256(robot.sensor.DR1count-old_sensor.DR1count)*drivecount2cm,
			wheelbase);
	}


// Send out telemetry
	static double last_send=0.0;
	if (cur_time>last_send+0.050)
	{
		last_send=cur_time;
		robotPrintln("Sending telemetry, waiting for command");
		telemetry.count++;
		telemetry.state=robot.state; // copy current values out for send
		telemetry.status=robot.status;
		telemetry.sensor=robot.sensor;
		telemetry.power=robot.power;
		telemetry.loc=robot.loc; robot.loc.confidence*=0.99;

		comms.broadcast(telemetry);
	}

// Show real and simulated robots
	robot_display(robot.loc);

	static double last_time=0.0;
	double dt=cur_time-last_time;
	if (dt>0.1) dt=0.1;
	last_time=cur_time;

	if (robot.loc.confidence>0.5)  // make sim track reality
		blend(sim.loc,robot.loc,robot.loc.confidence*dt);
	if (simulate_only) // make reality track sim
	{
		robot.loc=sim.loc; // blend(robot.loc,sim.loc,0.1);
		if (fabs(sim.loc.angle)<40.0) // camera in view
			robot.loc.confidence+=0.1;
		else // camera not in view
			robot.loc.confidence*=0.9;
		robot.loc.confidence*=0.9;

		if (robot.loc.y>100 && robot.loc.y<500) { // simulate obstacles
		if ((rand()%1000)==0) { // crater!
			sim.loc.angle+=30.0;
		}
		if ((rand()%1000)==0) { // crater!
			sim.loc.angle-=30.0;
		}
		}
	}
	sim.simulate(robot.power,dt);

	robot_display(sim.loc,0.5);

// Show path planning
  if (simulate_only) { //<- fixme: move path planning to dedicated thread, to avoid blocking
    // Show path back to dump
    vec2 shift(field_x_size/2.0,0.0);
    vec2 target(0,40);
    if (robot.state>=state_align_turnout && robot.state<state_drive_to_dump)
      target=vec2(0.0,field_y_size-100); // target is mining area

    // Start position: robot's position
    rmc_navigator::fposition fstart(robot.loc.x+shift.x,robot.loc.y+shift.y,90-robot.loc.angle);
    // End position: at target
    rmc_navigator::fposition ftarget(target.x+shift.x,target.y+shift.y,90);

    rmc_navigator::planner plan(navigator.navigator,fstart,ftarget,false);
    glBegin(GL_LINE_STRIP);
    for (const rmc_navigator::searchposition &p : plan.path)
    {
      //std::cout<<"Plan position: "<<p.pos<<" drive "<<p.drive<<"\n";
      glColor3f(0.5f+0.5f*p.drive.forward,0.5f+0.5f*p.drive.turn,0.0f);
      vec2 v=p.pos.v-shift;
      glVertex2fv(v);
    }
    glEnd();


  }
}


void display(void) {
	robot_display_setup(robot_manager->robot);

	robot_manager->update();

	glTranslatef(field_x_GUI+550.0,100.0,0.0);
	glScalef(300.0,200.0,1.0);
	glBindTexture(GL_TEXTURE_2D,video_texture_ID);
	glEnable(GL_TEXTURE_2D);
	glBegin(GL_QUAD_STRIP);
	glTexCoord2f(0.0,0.0); glVertex2f(0.0,0.0);
	glTexCoord2f(1.0,0.0); glVertex2f(+1.0,0.0);
	glTexCoord2f(0.0,1.0); glVertex2f(0.0,+1.0);
	glTexCoord2f(1.0,1.0); glVertex2f(+1.0,+1.0);
	glEnd();
	glBindTexture(GL_TEXTURE_2D,0);

	glutSwapBuffers();
	glutPostRedisplay();
}

int main(int argc,char *argv[])
{
	setenv("DISPLAY", ":0",1);
	glutInit(&argc,argv);

	// Set screen size
	int w=1280, h=700;
	for (int argi=1;argi<argc;argi++) {
		if (0==strcmp(argv[argi],"-sim")) {
			simulate_only=true;
			if (argi+1<argc) srand(atoi(argv[++argi])); // optional seed argument
			else srand(1);
		}
		else if (0==strcmp(argv[argi],"-big")) {
			big_field=true;
		}
		else if (0==strcmp(argv[argi],"-nodrive")) {
			nodrive=true;
		}
		else if (2==sscanf(argv[argi],"%dx%d",&w,&h)) {}
		else printf("Unrecognized argument '%s'!\n",argv[argi]);
	}
	glutInitDisplayMode(GLUT_RGBA + GLUT_DOUBLE);
	glutInitWindowSize(w,h);
	glutCreateWindow("Robot Backend");

	robot_manager=new robot_manager_t;
	robot_manager->robot.loc.y=field_y_start_zone/2;
	robotMainSetup();

	glutDisplayFunc(display);
	glutMainLoop();
	return 0;
}

