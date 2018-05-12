/**
  Navigation grid for Robotic Mining Competition
*/
#ifndef __bitgrid_RMC_H
#define __bitgrid_RMC_H


#include "gridnav/gridnav_RMC.h"

/**
 Store a simple on/off grid.
*/
class bitgrid {
public:
  typedef rmc_navigator nav;
  
  typedef unsigned char storage_t;
  enum {STORE_BITS=8};
  enum {NDATA=(nav::GRIDX*nav::GRIDY+STORE_BITS-1)/STORE_BITS};
  // This is a nav::GRIDY * nav::GRIDX raster bit image.
  storage_t data[NDATA];
  
  bitgrid() {
    for (int idx=0;idx<NDATA;idx++) data[idx]=0;
  }
  
  // Return true if these grid coords are in bounds:
  bool inbounds(int x,int y) const {
    if (x<0 || x>=nav::GRIDX) return false;
    if (y<0 || y>=nav::GRIDY) return false;
    return true;
  }
  
  // Read an in-bounds cell:
  bool read(int x,int y) const {
    int idx=y*nav::GRIDX+x;
    storage_t  d=data[idx/STORE_BITS];
    storage_t bit=1<<(idx%STORE_BITS);
    if (d&bit) return true;
    else return false;
  }
  
  // Write an in-bounds cell:
  void write(int x,int y,bool v) {
    int idx=y*nav::GRIDX+x;
    storage_t &d=data[idx/STORE_BITS];
    storage_t bit=1<<(idx%STORE_BITS);
    if (v) d |= bit; // turn on bit
    else   d &= ~bit; // turn off bit
  }
  
  // Print this grid to the screen:
  void print(void) {
    for (int y=nav::GRIDY-1;y>=0;y-=2)
    {
      for (int x=0;x<nav::GRIDX;x++) {
        if (x==0 || x==nav::GRIDX-1 || y==1 || y==nav::GRIDY-1) printf("+");
        else if (read(x,y) || read(x,y-1)) printf("#");
        else printf(" ");
      }
      printf("\n");
    }
  }
};

#endif


