/*
  Gridnav library: simple test driver program.
*/
#include "gridnav_RMC.h"

int main() 
{
  std::cout<<"Setting up grid\n";
  // Make RMC navigator
  rmc_navigator nav(false);
  
  std::cout<<"Setting up obstacles\n";
  // Make some obstacles on left side
  for (int x=0;x<150;x++) nav.mark_obstacle(x,300,40);
  // Make a straddleable obstacle in middle
  // for (int x=150;x<250;x++) nav.mark_obstacle(x,300,10);
  
  // Plan a path
  std::cout<<"Planning path\n";
  rmc_navigator::fposition start(70,600,20);
  rmc_navigator::fposition target(190,40,90);
  
  rmc_navigator::planner plan(nav.navigator,start,target,false);
  for (const rmc_navigator::searchposition &p : plan.path)
    std::cout<<"Plan position: "<<p.pos<<" drive "<<p.drive<<"\n";
  
  nav.navigator.lastpath.print();
  
  return 0;
}

