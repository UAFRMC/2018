/**
 Robot simulator, used for path planning, software development, and pilot training.
 
 Orion Lawlor, lawlor@alaska.edu, 2014-04-18 (public domain)
*/
#ifndef __AURORA_SIMULATOR_H
#define __AURORA_SIMULATOR_H

#include "../osl/vec4.h"
#include "../osl/vec2.h"

void blend_angles(float &dest,const float &src,float weight) {
	if(fabs(src - dest) > 180) { // must reduce source to match dest
		float sa=src;
		while (sa>dest+180) sa-=360;
		while (sa<dest-180) sa+=360;
		dest=sa*weight+dest*(1.0-weight);
		while (dest>180) dest-=360;
		while (dest<-180) dest+=360;
	} else {
		dest=src*weight+dest*(1.0-weight);
	}
}

void blend(robot_localization &dest, const robot_localization &src, float weight) {
	dest.x=src.x*weight+dest.x*(1.0-weight);
	dest.y=src.y*weight+dest.y*(1.0-weight);
	blend_angles(dest.angle,src.angle,weight);
	blend_angles(dest.pitch,src.pitch,weight);
}

class robot_simulator {
public:
	// Actuators:
	double dump; // linear actuators, 0-1 range
	double Mcount; // mining head counter
	double Rcount; // roll motor
	double bucket; // linear actuators, 0-1 range
	robot_localization loc; // current location of robot
	
	robot_simulator() {
		dump=0.0; // lowered
		bucket=0.5; // lowered
		Mcount=0;
		Rcount=0;
	}

/* Coordinate systems */
	/** Return the robot's orientation angle, in radians.  
	    0 is facing the lunabin.  + is clockwise; - is counterclockwise.
	*/
	double angle_rad() const {
		return loc.angle*M_PI/180.0;
	}
	/** Return the robot's forward (+y) unit direction vector. */
	vec2 forward() const {
		double a=angle_rad();
		return vec2(sin(a),cos(a));
	}
	/** Return the robot's right (+x) unit direction vector. */
	vec2 right() const {
		double a=angle_rad();
		return vec2(cos(a),-sin(a));
	}
	/** Convert this robot-coordinates location to world coordinates (in cm) */
	vec2 world_from_robot(const vec2 &robot_coords) const {
		return vec2(loc.x,loc.y)+forward()*robot_coords.y+right()*robot_coords.x;
	}
	/** Convert this world-coordinates location to robot coordinates (in cm) */
	vec2 robot_from_world(const vec2 &world_coords) const {
		vec2 rel=world_coords-vec2(loc.x,loc.y);
		return vec2(dot(right(),rel),dot(forward(),rel));
	}

	enum {wheelbase=65}; // left-right centimeters from centerline to wheel drive point (X)
	enum {wheelfront=45}; // front-back centimeters between axles (Y)
	enum {wheelforward=0}; // centimeters from center of mass to drive center (Y)

/* Return the world-coordinates location of this corner of the robot. */
	vec2 corner(bool right,bool front) {
		return world_from_robot(vec2(right?+wheelbase:-wheelbase, front?+wheelfront:-wheelfront));
	}

/* Simulate these robot power values, for this timestep (seconds) */
	void simulate(const robot_power &power, double dt) {
	// Move both wheels
		vec2 side[2];  // Location of wheels:  0: Left; 1:Right
		side[0]=world_from_robot(vec2(-wheelbase,wheelforward));
		side[1]=world_from_robot(vec2(+wheelbase,wheelforward));

		float sidepower[2];
		float topspeed=80.0; // <- speed in cm/sec at 100% power (hypothetical!)
		sidepower[0]=power.left-64.0;
		sidepower[1]=power.right-64.0;

		for (int s=0;s<2;s++) {
			double torque=sidepower[s]; // wheel torque command
			if (fabs(torque)>2.0) { // friction
				double distance=torque/64*topspeed*dt;
				side[s]+=distance*forward();
			}
		}
		
	// Set robot position and orientation from wheel positions
		vec2 center=(side[0]+side[1])*0.5;
		loc.x=center.x; loc.y=center.y;
		vec2 right=side[1]-side[0];
		loc.angle=atan2(-right.y,right.x)*180.0/M_PI;
	
	// Update bag roll counter
	  float Rcount_per_sec=300.0; // roll motor counts/sec at full speed
  	Rcount+=dt*Rcount_per_sec*(power.roll-64.0)/64.0;
  	
	// Update mining head counter
		float Mcount_per_sec=100.0; // mining head counts/sec at max speed
		float Mpower=(power.mine-64.0)/64.0;
		if (power.mineDump) Mpower=-0.3;
		if (power.mineMode) Mpower=0.6;
		Mcount+=dt*Mpower*Mcount_per_sec;
		while (Mcount>120.0) Mcount-=120.0;
		while (Mcount<0.0) Mcount+=120.0;
	
	// Update linear actuators
		double linear_scale=1.0/7.0/64.0; // seconds to full deploy, and power scale factor
		
		bucket+=dt*(power.dump-64.0)*linear_scale;
		if (bucket<0.0) bucket=0.0; 
		if (bucket>1.0) bucket=1.0;
	}
};




#endif


