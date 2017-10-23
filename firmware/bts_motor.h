#ifndef BTS_MOTOR_H
#define BTS_MOTOR_H

#include <Arduino.h>

// Manages a BTS motor controller
class BTS_motor_t
{
public:
  int L,R; // pins for left and right control channels
  int max_power;
  BTS_motor_t(int L_,int R_,int max_power_) :
  L(L_), R(R_), max_power(max_power_) {
    stop();
  }

  void stop() {
    analogWrite(L,0);
    analogWrite(R,0);
  }

  /* Drive from Sabertooth-style command:
   64 means stop
   1 means full reverse
   127 means full forward
   */
  void drive(int speed64) {
    int speed255=(speed64-64)*4;
    
    if(speed255<-max_power)
      speed255=-max_power;
    if(speed255>max_power)
      speed255=max_power;
    
    
    if (speed255<0) {
      analogWrite(L,-speed255);
      analogWrite(R,0);
    }
    else {
      analogWrite(L,0);
      analogWrite(R,speed255);
    }
  }
};

#endif
