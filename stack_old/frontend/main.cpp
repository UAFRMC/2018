/**
  Aurora Robotics frontend Code
  
  Orion Sky Lawlor, lawlor@alaska.edu, 2014-03-23 (Public Domain)
*/
#define AURORA_IS_FRONTEND 1

#include <iostream>
#include <cmath>

#include "osl/quadric.h"
#include "../../firmware/robot_base.h"
#include "aurora/robot.cpp"
#include "aurora/display.h" /* for graphics */
#include "aurora/network.h"
#include "aurora/ui.h"

#include "ogl/event.cpp"
#include "osl/socket.cpp"

#include "osl/porthread.h" /* for threading */
#include "osl/porthread.cpp"

/**
 This class represents everything the front end knows about the robot.
*/
class robot_manager_t 
{
public:
	robot_base robot; // overall integrated current state
	
	robot_telemetry telemetry; // last-known telemetry value
	byte last_telemetry_count;
	double last_telemetry_time;
	
	robot_ui ui; // keyboard interface
	robot_command command; // next-sent command
	double last_command_time;
	robot_comms comms; // network link to back end
	
	
	// Do robot work.
	void update(void);
	
	robot_manager_t() {
		last_telemetry_count=0;
		last_telemetry_time=0;
		last_command_time=0;
	}
};
robot_manager_t robot_manager;

void robot_manager_t::update(void) {
	double time=robotTime();
	robotPrintln("Location %.0f,%0.0f,%0.0f",robot.loc.x,robot.loc.y,robot.loc.angle);
	
// Run UI
	ui.update(oglKeyMap,robot);
	robotPrintLines(ui.description);
	if (time>=last_command_time+0.050) 
	{
		if (robotState_requested<state_last)
		{ // request to enter state
			command.command=robot_command::command_state;
			command.power=ui.power;
			command.state=robotState_requested;
			robotPrintln("REQUESTING ROBOT STATE %s",state_to_string(robotState_requested));
		} 
		else 
		{ // normal powered driving
			command.command=robot_command::command_power;
			command.power=ui.power;
			command.state=state_drive;
		}
		comms.broadcast(command);

		if (robot.state==state_drive) 
		{
			robot.power=ui.power;
		}
		
		last_command_time=robotTime();
	}
	
// Check for a telemetry broadcast
	int n;
	while (0!=(n=comms.available(10))) {
		time=robotTime();
		if (n==sizeof(telemetry)) 
		{ // grab telemetry from backend
			comms.receive(telemetry);
			
			robot.state=(robot_state_t)telemetry.state;
			
			static int last_state=robot.state;
			if (last_state!=robot.state)
				robotPrintln("Robot entering state %s",state_to_string(robot.state));
			last_state=robot.state;
			
			if (robotState_requested<state_last)
			{ // we asked for a new state--stop asking once it's confirmed
				if (telemetry.ack_state==robotState_requested)
				{
					robotState_requested=state_last; // confirmed.
				}
			}
			
			robot.status=telemetry.status;
			robot.sensor=telemetry.sensor;
			robot.loc=telemetry.loc;
			if (robot.state!=state_drive) 
			{ // show autonomous mode power
				robot.power=telemetry.power;
				 
			}	
			
			robotPrintln("Telemetry: state %s (%d bytes)",
				state_to_string((robot_state_t)telemetry.state),
				sizeof(telemetry));
			byte next_count=1+last_telemetry_count;
			if (telemetry.count!=next_count && next_count!=1) {
				robotPrintln("Telemetry warning> count mismatch. Expected %d, got %d",
					next_count,telemetry.count);
			}
			if (time>0.15+last_telemetry_time) {
				robotPrintln("Telemetry warning> time gap of %.3f seconds",
					time-last_telemetry_time);
			}
			
			last_telemetry_count=telemetry.count;
			last_telemetry_time=time;
			
		} 
		else {
			robotPrintln("ERROR: TELEMETRY VERSION MISMATCH!  Expected %d or %d, got %d",
				sizeof(telemetry),sizeof(command),n);
		}
	}
	/*
	else {
		robotPrintln("NO TELEMETRY");
	}*/
	
	robot_display(robot.loc);
}

extern "C" void display(void) {
	robot_display_setup(robot_manager.robot);
	
	robot_manager.update();
	
	glutSwapBuffers();
	glutPostRedisplay();
}


int main(int argc,char *argv[]) 
{
	glutInit(&argc,argv);
	
	// Set screen size
	int w=1280, h=700;
	for (int argi=1;argi<argc;argi++) {
		if (0==strcmp(argv[argi],"-bench")) {  }
		else if (0==strcmp(argv[argi],"-img")) {  }
		else if (2==sscanf(argv[argi],"%dx%d",&w,&h)) {}
		else printf("Unrecognized argument '%s'!\n",argv[argi]);
	}
	glutInitDisplayMode(GLUT_RGBA + GLUT_DOUBLE);
	glutInitWindowSize(w,h);
	glutCreateWindow("Robot Front End");
	
	robotMainSetup();
	
	glutDisplayFunc(display);
	glutMainLoop();
	return 0;
}
