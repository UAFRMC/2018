/**
 Aurora Robotics OpenGL display code.
 Shared between front end and back end.
*/
#ifndef __AURORA_ROBOTICS__DISPLAY_H
#define __AURORA_ROBOTICS__DISPLAY_H

#include <GL/glut.h> /* OpenGL Utilities Toolkit, for GUI tools */

/* robotPrintln support code: */
#include <stdio.h>
#include <stdarg.h>  /* for varargs stuff below */
#include "osl/vec2.h"
#include "osl/quadric.h"
#include <string>

robot_state_t robotState_requested=state_last;
vec2 robotMouse_pixel; // pixel position of robot mouse
vec2 robotMouse_cm; // field-coordinates position of mouse
bool robotMouse_down=false;

double robotPrintf_x=field_x_GUI, robotPrintf_y=0.0, robotPrintf_line=-25.0;

/* Return the current time, in seconds */
double robotTime(void) {
	return 0.001*glutGet(GLUT_ELAPSED_TIME);
}


bool robotPrintf_enable=true;
/* Render this string at this X,Y location */
void robotPrint(float x,float y,const char *str)
{
	if (robotPrintf_enable) {
        // Dump everything to the console, and log it too
	fprintf(stdout,"%.3f %s\n",robotTime(),str);
        fflush(stdout);

        static FILE *flog=fopen("log.txt","w");
	fprintf(flog,"%.3f %s\n",robotTime(),str);
        fflush(flog);
        }

        // Draw it onscreen
        void *font=GLUT_BITMAP_HELVETICA_12;
        glRasterPos2f(x,y);
        while (*str!=0) {
        	glutBitmapCharacter(font,*str++);
        	if (robotPrintf_enable && (*str=='\n' || *str==0)) {
        		robotPrintf_x=field_x_GUI;
        		robotPrintf_y+=robotPrintf_line;
        	}
        }
        glPopAttrib();

}

/** Render this string onscreen, followed by a newline. */
void robotPrintln(const char *fmt,...) {
        va_list p; va_start(p,fmt);
        char dest[1000];
        vsnprintf(dest,sizeof(dest),fmt,p);
        robotPrint(robotPrintf_x,robotPrintf_y,dest);
        va_end(p);
}

void robotPrintLines(const std::string& text)
{
	std::string temp;

	for(size_t ii=0;ii<text.size();++ii)
	{
		if(text[ii]=='\n'||ii+1>=text.size())
		{
			robotPrintln(temp.c_str());
			temp="";
			continue;
		}

		temp+=text[ii];
	}
}

/******************************** GUI ****************************/

// Rotate vector src around the origin by this many degrees
inline vec2 rotate(const vec2 &src,float ang_deg) {
	double ang_rad=ang_deg*M_PI/180.0;
	double s=sin(ang_rad), c=cos(ang_rad);
	return vec2( c*src.x-s*src.y, s*src.x+c*src.y);
}

inline float state_to_Y(int state) {
	return 0+field_y_size*(state_last-state)*(1.0/state_last);
}

/* Called at start of user's OpenGL display function */
void robot_display_setup(const robot_base &robot) {

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_ALPHA_TEST);
	glDisable(GL_BLEND);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	int wid=glutGet(GLUT_WINDOW_WIDTH), ht=glutGet(GLUT_WINDOW_HEIGHT);
	glViewport(0,0,wid,ht);

	// Encode current robot state in background color:
	if (robot.state==state_STOP) {
		glClearColor(0.7,0.0,0.0,0.0); // bright red (STOP sign)
	}
	else if (robot.state==state_drive) {
		glClearColor(0.3,0.3,0.3,0.0); // drive: dim gray
	}
	else {
		glClearColor(0.1,0.2,0.4,0.0); // blue autonomy
	}

	glClear(GL_COLOR_BUFFER_BIT+GL_DEPTH_BUFFER_BIT);

	// Scale to showing the whole field, in centimeter units
	float xShift=-0.5, yShift=-0.85; // GL-coordinates start of field
	glTranslatef(xShift,yShift,0.0);
	float yScale=1.8/field_y_size;
	float xScale=yScale*ht/wid;
	glScalef(xScale, yScale, 0.1);
	robotPrintf_y=(1.0-yShift)/yScale+robotPrintf_line;

	// Read back the matrix to get from cm to onscreen pixels
	float mat[16];
	glGetFloatv(GL_MODELVIEW_MATRIX,mat);
	int w=glutGet(GLUT_WINDOW_WIDTH), h=glutGet(GLUT_WINDOW_HEIGHT);
	vec2 mat_scale(1.0/mat[0],1.0/mat[5]);
	vec2 mat_offset(mat[12],mat[13]);

	// coordinate-convert mouse to cm coords
	vec2 m=vec2(robotMouse_pixel.x*2.0/w-1.0,(h-robotMouse_pixel.y)*2.0/h-1.0)-mat_offset;
	m.x*=mat_scale.x;
	m.y*=mat_scale.y;
	robotMouse_cm=m;

	glLineWidth(1+3*wid/1000);

	/*
	glBegin(GL_LINES); // to verify mouse position
	glColor4f(0.0,0.6,0.0,1.0);
	glVertex2fv(robotMouse_cm);
	glVertex2fv(robotMouse_cm+vec2(10,20));
	glEnd();
	*/

// Delineate the start and mine zones
	glBegin(GL_LINES);
	glColor4f(0.3,0.3,0.5,1.0);
	glVertex2i(-field_x_hsize,field_y_start_zone);
	glVertex2i(+field_x_hsize,field_y_start_zone);
	glVertex2i(-field_x_hsize,field_y_mine_zone);
	glVertex2i(+field_x_hsize,field_y_mine_zone);

// Draw the lunabin
	glColor4f(0.3,1.0,1.0,1.0);
	glVertex2i(-field_x_hbin,-10);
	glVertex2i(+field_x_hbin,-10);
	glEnd();

// Outline the field
	glBegin(GL_LINE_LOOP);
	glColor4f(0.0,0.0,0.8,1.0);
	glVertex2i(+field_x_hsize,0);
	glVertex2i(-field_x_hsize,0);
	glVertex2i(-field_x_hsize,field_y_size);
	glVertex2i(+field_x_hsize,field_y_size);
	glEnd();


// Draw xmitters
	glBegin(GL_LINES);
	for (int xmit=0;xmit<3;xmit++) {
		float color[4]={0.2,0.2,0.2,1.0};
		color[xmit%3]=1.0;
		glColor4fv(color);
		vec2 start((xmit-1)*field_x_xmit,field_y_xmit);
		glVertex2fv(start);
		glVertex2fv(start+vec2(5.0,0.0));
	}
	glEnd();

// Draw current robot configuration (side view)
	glBegin(GL_TRIANGLES);
	double robot_draw_y=75; // size of side view image
	double robot_draw_x=-75;
	vec2 robot_draw(0.6*field_x_size-robot_draw_x,200);
	vec2 dump_pivot=robot_draw;

	glColor4f(0.0,0.0,0.0,1.0); // body (black)
	glVertex2fv(robot_draw);
	glVertex2fv(robot_draw+vec2(robot_draw_x,0));
	glVertex2fv(robot_draw+vec2(0,robot_draw_y));

	double dump_angle=-30.0*((robot.sensor.bucket-180.0)/(950.0-180.0))+10.0;
	
	if (robot.loc.pitch!=0.0) {
	  robotPrintln("Robot pitch: %.1f deg\n",robot.loc.pitch);
	  dump_angle=-robot.loc.pitch;
	}

	vec2 dump_tip=dump_pivot + rotate(vec2(0,robot_draw_y-10),dump_angle);
	vec2  box_tip=dump_pivot + rotate(vec2(0,15),dump_angle);
	vec2 mine_tip=dump_pivot + rotate(vec2(robot_draw_x*0.8,0),dump_angle);

	glColor4f(0.0,0.0,0.0,1.0); // body (black)
	glVertex2fv(dump_pivot);
	glColor4f(0.0,1.0,0.0,0.5); // Dump bin (green)
	glVertex2fv(dump_tip);
	glColor4f(1.0,0.0,0.0,0.5); // Tip of mining head (red)
	glVertex2fv(mine_tip);

	// Graphical illustration of Mcount:
	vec2 mine1=mine_tip;
	vec2 mine0=dump_tip;
	float Mprogress=((robot.sensor.McountL+119)%120)/120.0*0.8;
	vec2 Mprog=mine1+Mprogress*(mine0-mine1);
	glColor4f(1.0,0.0,0.0,1.0);
	glVertex2fv(Mprog);
	glVertex2fv(Mprog+rotate(vec2(0,20),dump_angle));
	glVertex2fv(Mprog+rotate(vec2(-20,0),dump_angle));
	glEnd();
	
	// Graphical illustration of the dust storage box:
	vec2 box0=box_tip;
	vec2 box1=dump_tip;
	float Rprogress=((robot.sensor.Rcount-box_raise_min)/float(box_raise_max-box_raise_min));
	vec2 box=box0+Rprogress*(box1-box0);
	glColor4f(0.8,0.8,0.2,1.0);
	glBegin(GL_TRIANGLE_FAN);
	glVertex2fv(box);
	glVertex2fv(box+rotate(vec2(-10,0),dump_angle));
	glVertex2fv(box+rotate(vec2(-10,20),dump_angle));
	glVertex2fv(box+rotate(vec2(0,20),dump_angle));
	glEnd();

// Draw the current autonomy state
	robotPrintf_enable=false;
	double state_display_x=3*field_x_hsize;
	for (robot_state_t state=state_STOP;state<state_last;state=(robot_state_t)(state+1))
	{
		glColor4f(0.0,0.0,0.0,1.0); // black inactive

		if (state==robotState_requested || (
		    robotMouse_cm.x>state_display_x &&
		    robotMouse_cm.y<state_to_Y(state) &&
		    robotMouse_cm.y>state_to_Y(state+1)
		    ))
		{ // red mouse hover
			glColor4f(1.0,0.0,0.0,1.0);
			if (robotMouse_down==true)
			{ // request new state
				robotState_requested=state;
			}
		}

		if (state==robot.state) {
			glColor4f(1.0,1.0,1.0,1.0); // white when active
		}
		robotPrint(state_display_x,
			0.5*(state_to_Y(state)+state_to_Y(state+1)), // average height
			state_to_string(state));
	}
	robotPrintf_enable=true;

// Draw current robot power values
	unsigned char *powers=(unsigned char *)&robot.power; // HACK: want array of powers
	glBegin(GL_TRIANGLES);
	for (unsigned int i=0;i<sizeof(robot.power);i++) {
		unsigned int pow=powers[i]&0x7f;
		int autonomous=powers[i]&0x80;
		float cenx=50*(0.5+i)+field_x_GUI;
		float ceny=0.10*field_y_size;
		glColor3ub(128+pow,autonomous?255:128,255-pow);
		glVertex2f(cenx-20,ceny);
		glVertex2f(cenx+20,ceny);
		glVertex2f(cenx,ceny+2.0*(powers[i]-60));
	}
	glEnd();

// Output telemetry as text (for log, mostly)
	glColor3f(1.0,1.0,1.0);

	robotPrintln("Left/Right Mining Motor Counts: %d, %d",robot.sensor.McountL, robot.sensor.McountR);
	robotPrintln("Track front encoder ticks %d L %d R", robot.sensor.DL1count, robot.sensor.DR1count);
	robotPrintln("Track back encoder ticks %d L %d R", robot.sensor.DL2count, robot.sensor.DR2count);
	robotPrintln("Roll motor encoder ticks %d", robot.sensor.Rcount);
	robotPrintln("\"The Box\" limit ticks %d %d", robot.sensor.limit_top, robot.sensor.limit_bottom);


	std::string box_status = "";

	if(robot.sensor.Rcount <= box_raise_min)
	{
		box_status = "lowered";
	}
	else if(robot.sensor.Rcount >= box_raise_max)
	{
		box_status = "raised";
	}
	else
	{
		box_status = "in motion";
	}
	robotPrintln("\"The Box\" is %s", box_status.c_str());


	std::string encoder_str("Encoder Raw ");
	for(int ii=12-1;ii>=0;--ii)
	{
		if((robot.sensor.encoder_raw&(1<<ii))!=0)
			encoder_str+="1";
		else
			encoder_str+="0";
		if(ii==6)
			encoder_str += " ";
	}
	robotPrintln(encoder_str.c_str());

	if (robot.status.arduino)
	{ // arduino connected: print status
		std::string status="";
		if (robot.status.stop) status+="STOP(status) ";
		if (robot.sensor.stop) status+="STOP(sensor) ";
		if (robot.status.located) status+="located ";
		if (robot.status.autonomy) status+="AUTONOMY ";
		if (robot.status.semiauto) status+="SEMIAUTO ";
		robotPrintln("Arduino connected: %s",status.c_str());

	// Analog voltage dividers:
	// Linear actuators:
		robotPrintln("  bucket %.1f%% (%d) up",
			(robot.sensor.bucket-179.0)*100.0/(920-179.0),robot.sensor.bucket);

		//robotPrintln("  battery %.2f V (%d)",
		//	robot.sensor.battery*AD_DN2high_voltage,robot.sensor.battery);

		robotPrintln("  MCU latency %d",
			robot.sensor.latency);
	} else {
		robotPrintln("Arduino not connected");
	}

	if (robot.loc.confidence>0.5) {
		robotPrintln("Location:  X %.1f   Y %.1f   angle %.0f",
			robot.loc.x*0.01,robot.loc.y*0.01,
			robot.loc.angle);
	}
}

void robot_display(const robot_localization &loc,double alpha=1.0)
{
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

// Draw the robot
	float conf=loc.confidence;
	glColor4f(0.8,0.8*conf,0.8*conf,alpha);
	glBegin(GL_TRIANGLE_FAN);
	//float ang=loc.angle*M_PI/180.0;
	vec2 C=loc.center(); // (loc.x,loc.y); // center of robot
	vec2 F=75.0/2*loc.forward(); // (+30.0*sin(ang), +30.0*cos(ang)); // robot forward direction
	vec2 R=128.0/2*loc.right(); // (+70.0*cos(ang), -70.0*sin(ang)); // robot right side
	double d=1.0; // front wheel deploy?

	glColor4f(0.0,0.8*conf,0.0,alpha); // green center
	glVertex2fv(C+0.5*F);

	glColor4f(0.8*conf,0.0,0.0,alpha); // red front wheels
	glVertex2fv(C-R+d*F);

	glColor4f(0.0,0.0,0.0,alpha); // black back
	glVertex2fv(C-R-F);
	glVertex2fv(C+R-F);

	glColor4f(0.8*conf,0.0,0.0,alpha); // red front wheels
	glVertex2fv(C+R+F);
	glEnd();

	glColor4f(1.0,1.0,1.0,1.0);
}


/*************************** Keyboard **********************************/
/** Handle keyboard presses */
#include "../ogl/event.h"

extern "C" void ogl_main_keyboard(unsigned char key, int x, int y)
{
        // flip toggles on keypress
        oglToggles[key]=!oglToggles[key];
        oglKeyMap[key]=1;
}
extern "C" void ogl_main_keyboard_up(unsigned char key, int x, int y)
{
        oglKeyMap[key]=0;
}
extern "C" void ogl_main_special(int key, int x, int y)
{
        if (key<0x80)
                oglKeyMap[0x80+key]=1;
}
extern "C" void ogl_main_special_up(int key, int x, int y)
{
        if (key<0x80)
                oglKeyMap[0x80+key]=0;
}

void ogl_mouse_motion(int x, int y) {
	robotMouse_pixel=vec2(x,y);
}

void ogl_mouse(int button,int state,int x,int y)
{ /* mouse being pressed or released--save position for motion */
	ogl_mouse_motion(x,y);
	if (state==GLUT_DOWN) {
	       robotMouse_down=true;
	} else {
	       robotMouse_down=false;
	}
}

void robotMainSetup(void) {
	glutKeyboardFunc (ogl_main_keyboard);
	glutKeyboardUpFunc (ogl_main_keyboard_up); /* "up" version for KeyMap */
	glutSpecialFunc (ogl_main_special); /* for arrow keys */
	glutSpecialUpFunc (ogl_main_special_up);
	glutMouseFunc(ogl_mouse);
	glutMotionFunc(ogl_mouse_motion);
	glutPassiveMotionFunc(ogl_mouse_motion);

}



#endif

