#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include <Arduino.h>
#include "encoder.h"
#include "bts_motor.h"

template<uint8_t AVERAGES> class speed_controller_t:public encoder_t
{
public:
  BTS_motor_t& motor;
  pid_t<int16_t> pid;
  milli_t last_print;
  milli_t last_power;
  milli_t times[AVERAGES];
  uint8_t times_ptr; // index into times array
  int16_t average_without_div; // average of times array
  milli_t target_time; // motor target period (ms)
  int last_dir;
  bool stalled;
  boolean backwards; // running in reverse

  speed_controller_t(const float pgain,const float igain,const float dgain,const int encoder_pin,int encoder_threshold,BTS_motor_t& motor):
    encoder_t(encoder_pin),motor(motor),pid(0,0,0),average_without_div(0),times_ptr(0),target_time(1000)
  {
    memset(times,0,sizeof(times));
    stalled=false;
    backwards=false;
    last_dir=0;
    last_print=last_power=0;
  }

  virtual void change()
  {
    times[times_ptr++]=period;
    if(times_ptr>=AVERAGES)
      times_ptr=0;

    average_without_div=0;
    for(int16_t ii=0;ii<AVERAGES;++ii)
      average_without_div+=times[ii];

    stalled=false; // evidently we're spinning!
  }

  // Update state for new commanded direction
  void set_dir(int dir) {
      backwards=dir<0;
      if (dir!=last_dir) {
        last_dir=dir;
        last_change=milli; // update timing for start of new command
      }
  }


  //target is either target pwm speed in speed control mode (when torque_control is false)
  //target is either target ms time in torque control mode (when torque_control is true)
  int update(int target,const bool speed_control,int rpm_scaler=6)
  {
    //speed control, return
    if(!speed_control) {
      last_change=milli;
      return target;
    }
    // target == 1 full reverse .. 64 stop .. 127 full forward

    //backwards calculation
    if(target<62) {
      set_dir(-1);
    }
    else if (target>=62 && target<65) {
      set_dir(0);
      stalled=false; //No power,clear stall
      return 64;
    }
    else { // target > 64
      set_dir(+1);
      target=128-target;
    }
   // target == 63 (stop) .. 0 (full power)

    //Change pwm speed to time in ms
    int rpm=(63-target)*rpm_scaler;
    if(rpm==0)
      rpm=1;
    target_time=2000/rpm;

    int min_time=2;
    int max_time=500;

    if(target_time>max_time)
      target_time=max_time;
    if(target_time<min_time)
      target_time=min_time;

    const int motor_min=68; // don't give less power than this (keep spinning)
    const int motor_max=127; // don't give more than this
    const int motor_stop=64;
    const int motor_idle=72;
    const int motor_kickstart_gentle=80;
    const int motor_kickstart_full=motor_max;
    int32_t motor_value=64;
    milli_t since_last=milli-last_change;

    if(last_power>last_change+500)
    {
       stalled=true;
    }
    
    
    {
      milli_t real_average=average_without_div;
      int count=AVERAGES;

      if(since_last>real_average/AVERAGES) // next tick is late--include it
      {
        real_average+=since_last;
        ++count;
      }

      real_average=real_average/count;

      int32_t err=((int32_t)real_average-(int32_t)target_time)*16/target_time;
      motor_value=err+motor_idle;

      // avoid spinning too slow
      if(motor_value<motor_min)
        motor_value=motor_min;
    }

    // avoid spinning too fast
    if(motor_value>motor_max)
      motor_value=motor_max;

// Debug prints
/*
    if(milli-last_print>=100)
    {
      Serial.print("MINE: ");
      Serial.print(average_without_div/AVERAGES);
      Serial.print("\t");
      Serial.println(motor_value);

      last_print=milli;
    }
*/

    if(motor_value!=64)
      last_power=milli;

    if(backwards) //running in reverse
      return (128-motor_value);
    else
      return motor_value;
  }
};

#endif
