/*
  Use the gridnav library for RMC-style robot.
*/
#ifndef AURURA_GRIDNAV_RMC_H
#define AURURA_GRIDNAV_RMC_H

#include "gridnav.h"

/**
  Build a gridnav navigator to plan paths for our 
  Robot Mining Competition sized robot.
  
  
*/
class rmc_navigator : public gridnav::robot_geometry {
public:
  // RMC-plausible navigation grid dimensions:
  enum {GRIDSIZE=10}; // cm per grid cell
  enum {GRIDX=(378+GRIDSIZE-1)/GRIDSIZE}; // xy grid cells for field
  enum {GRIDY=(738+GRIDSIZE-1)/GRIDSIZE};
  enum {GRIDA=36}; // angular slices around 360 degrees
  enum {ROBOTSIZE=(150+GRIDSIZE-1)/GRIDSIZE};

  // Create the navigator and planner
  typedef gridnav::gridnavigator<GRIDX, GRIDY, GRIDA, GRIDSIZE, ROBOTSIZE> navigator_t;
  navigator_t navigator;
  
  typedef navigator_t::planner planner;
  typedef navigator_t::fposition fposition;
  typedef navigator_t::searchposition searchposition;
  
  // Discretized version of our geometry
  navigator_t::robot_grid_geometry robot;
  
  // Build the navigator:
  rmc_navigator(bool verbose=false) :robot(*this,verbose)
  {
    navigator.mark_edges(robot);
  }
  
  // Add an obstacle at this (x,y), in centimeters
  void mark_obstacle(int x,int y,int height) {
    for (int dy=-GRIDSIZE;dy<=0;dy++)
    for (int dx=-GRIDSIZE;dx<=0;dx++)
      navigator.mark_obstacle((dx+x)/GRIDSIZE,(dy+y)/GRIDSIZE,height,robot);
  }


 // gridnav::robot_geometry interface:
    //    (0,0) is the robot's origin, at the center of turning.
    //    +x is the direction the robot naturally drives forward
  virtual int clearance_height(float x,float y) const 
  {
  // front-back:
    if (x>40.0 || x<-35.0) return gridnav::OPEN;
    if (y<0) y=-y; // apply Y symmetry
    if (y>75.0) return gridnav::OPEN; // beyond the tracks
    if (y>45.0) return 0; // tracks
    return 20; // open area under mining head
  }
};




#endif



