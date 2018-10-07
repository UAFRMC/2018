/**
 * Aurora Robotics "Autonomy" firmware - 2015 BTS Version
 * For Arduino Mega
 */
#include "robot_base.h" /* classes shared with PC, and across network */
#include "serial_packet.h" /* CYBER-Alaska packetized serial comms */
#include "encoder.h"
#include "milli.h"
#include "pid.h"
#include "bts_motor.h"
#include "speed_controller.h"

/* Subtle: I don't need a namespace, but this at least disables the Arduino IDE's
utterly broken prototype generation code in the Arduino IDE. */
namespace aurora {

int bts_enable_pin=22;
// Hardware pin wiring (for Mega)
BTS_motor_t motor_mining(10,11,255);
BTS_motor_t motor_box(12,3,255); // Box raise/lower motor
BTS_motor_t motor_drive_left(8,9,60);
BTS_motor_t motor_drive_right(5,4,60);
BTS_motor_t motor_side_linears(7,6,255);
BTS_motor_digital_t motor_front_linear(31,33,255);

// Call this function frequently--it's for minimum-latency operations
void low_latency_ops();

/** This class manages communication via an A_packet_formatter,
 including timeouts. */
class CommunicationChannel {
public:
  HardwareSerial &backend;
  A_packet_formatter<HardwareSerial> pkt; // packet formatter
  bool is_connected; // 1 if we're recently connected; 0 if no response
  milli_t last_read; // millis() the last time we got data back
  milli_t next_send; // millis() the next time we should send off data

  CommunicationChannel(HardwareSerial &new_backend) :
  backend(new_backend), pkt(backend)
  {
    is_connected=0;
    last_read=milli;
    next_send=milli;
  }

  bool read_packet(A_packet &p) {
    p.valid=0;
    if (backend.available()) {
      while (-1==pkt.read_packet(p)) {
        low_latency_ops(); /* while reading packet */
      }
      if (p.valid && p.length>0) {
        last_read=milli;
        next_send=milli;
        is_connected=true; // got valid packet
        //digitalWrite(13,HIGH); // !!(milli&(1<<8))); // blink while getting good data
        return true;
      }
    }
    if (milli-next_send>500) { // read timeout
      next_send=milli;
      pkt.reset();
      pkt.write_packet(0,0,0); // send heartbeat ping packet
      is_connected=false;

      //digitalWrite(13,0); // LED off if disconnected
    }
    return false;
  }
};

// All PC commands go via this (onboard USB) port
HardwareSerial &PCport=Serial; // direct PC
HardwareSerial &BlinkyLport=Serial1; // serial comms to blinky nanos
HardwareSerial &BlinkyRport=Serial2; // serial comms to blinky nanos

CommunicationChannel PC(PCport);
CommunicationChannel BlinkyL(BlinkyLport);
CommunicationChannel BlinkyR(BlinkyRport);

const int encoder_raw_pin_count=12;

const int encoder_raw_pins[encoder_raw_pin_count]={48,34,50,32,52,30, 42,40,44,38,46,36};

const int encoder_bus_1[9]={0,0,0, 48,34,50,32,52,30}; //bus 1-4
const int encoder_bus_2[9]={0,0,0, 42,40,44,38,46,36}; //bus 5-8

enum
{
  NUM_AVERAGES=2
};
//speed_controller_t<NUM_AVERAGES> encoder_M(4,0,0,encoder_bus_2[3],80,motor_mining); // Mining head left side motor

speed_controller_t<NUM_AVERAGES> encoder_mining_left(4,0,0,encoder_raw_pins[4],80,motor_mining);
speed_controller_t<NUM_AVERAGES> encoder_mining_right(4,0,0,encoder_raw_pins[11],80,motor_mining);

speed_controller_t<NUM_AVERAGES> encoder_R(4,0,0,encoder_raw_pins[6],80,motor_box); // Encoder for roll motor

// Only one drive encoder per side
speed_controller_t<NUM_AVERAGES> encoder_DL1(4,0,0,encoder_raw_pins[5],13,motor_drive_left);  //Left front wheel encoder
speed_controller_t<NUM_AVERAGES> encoder_DL2(4,0,0,encoder_raw_pins[5],13,motor_drive_left);  //Left back wheel encoder
speed_controller_t<NUM_AVERAGES> encoder_DR1(4,0,0,encoder_raw_pins[8],13,motor_drive_right); //Right front wheel encoder
speed_controller_t<NUM_AVERAGES> encoder_DR2(4,0,0,encoder_raw_pins[8],13,motor_drive_right);  //Right back wheel encoder

encoder_t limit_top(encoder_raw_pins[3]);
encoder_t limit_bottom(encoder_raw_pins[2]);





/***************** Robot Control Logic ****************/

long mine_last=0; // millis() at last mining motion check
long stall_last=0; // timer for stall detection average
int sum_stalled=0, sum_stall_count=0;


// Robot's current state:
robot_base robot;

// Read all robot sensors into robot.sensor
void read_sensors(void) {
  robot.sensor.battery=0; 
  low_latency_ops();

  robot.sensor.Mstall=encoder_mining_right.stalled; //encoder_mining_left.stalled || encoder_mining_right.stalled;
  robot.sensor.DRstall=encoder_DL1.stalled;
  robot.sensor.DLstall=encoder_DR1.stalled;
  robot.sensor.limit_top=limit_top.count_mono;
  robot.sensor.limit_bottom=limit_bottom.count_mono;
  ++robot.sensor.heartbeat;
  
  
  robot.sensor.encoder_raw=0;
  for(int ii=0;ii<encoder_raw_pin_count;++ii)
    robot.sensor.encoder_raw|=digitalRead(encoder_raw_pins[ii])<<ii;
}


void set_direction(int power64, encoder_t &enc)
{
  //Update encoder direction
  if (power64>64) enc.last_dir=+1;
  if (power64<64) enc.last_dir=-1;

}
// Match up motor power value with encoder
void send_motor_power(int power64,BTS_motor_t &motor,encoder_t &enc) {
  // update encoder direction
  set_direction(power64,enc);
  // send to motor
  motor.drive(power64);
}

// Send current power values to the motors
void send_motors(void)
{
  if(robot.power.motorControllerReset!=0)
    digitalWrite(bts_enable_pin,LOW);
  else
    digitalWrite(bts_enable_pin,HIGH);
 
  int drivePower=100;
  if(robot.power.high)
  {
    drivePower=255;
  }
  motor_drive_left.max_power=drivePower;
  motor_drive_right.max_power=drivePower;
  
  int left1=encoder_DL1.update(robot.power.left,robot.power.torqueControl==0);
  send_motor_power(left1,motor_drive_left,encoder_DL1);
  set_direction(left1,encoder_DL2);

  int right1=encoder_DR1.update(robot.power.right,robot.power.torqueControl==0);
  send_motor_power(right1,motor_drive_right,encoder_DR1);
  set_direction(right1,encoder_DR2);


  int roll = encoder_R.update(robot.power.roll,robot.power.torqueControl==0);
  send_motor_power(robot.power.roll,motor_box,encoder_R);

  // ***** Automated mining at fixed rate ***** //
  int mine_target=robot.power.mine;

  // Update left and right speed controllers
  const int mining_rpm_scalar = 100;
  //int mine_target_left = encoder_mining_left.update(mine_target,robot.power.mineMode!=0||robot.power.mineDump!=0);
  int mine_target_left=encoder_mining_left.update(mine_target,robot.power.torqueControl==0,mining_rpm_scalar);
  //int mine_target_right = encoder_mining_right.update(mine_target,robot.power.mineMode!=0||robot.power.mineDump!=0);
  int mine_target_right=encoder_mining_right.update(mine_target,robot.power.torqueControl==0,mining_rpm_scalar);

  // Send lowest speed set by speed controllers
  // TODO: FIX FOR BACKWARDS DRIVE
  int mine_target_final;
  //if(mine_target_left <= mine_target_right) {
  //  mine_target_final = mine_target_left;
  //}
  //else {
    mine_target_final = mine_target_right;
  //}
  send_motor_power(mine_target_final,motor_mining,encoder_mining_right);
  // ********** //

  // Send power to linears
  motor_front_linear.drive(robot.power.head_extend);
  motor_side_linears.drive(robot.power.dump);
}

// Structured communication with PC:
void handle_packet(A_packet_formatter<HardwareSerial> &pkt,const A_packet &p)
{
  if (p.command==0x7) { // motor power commands
    low_latency_ops();
    if (!p.get(robot.power)) { // error
      pkt.write_packet(0xE,0,0);
    }
    else
    { // got power request successfully: read and send sensors
      low_latency_ops(); /* while reading sensors */
      read_sensors();
      low_latency_ops();
      pkt.write_packet(0x3,sizeof(robot.sensor),&robot.sensor);
      robot.sensor.latency=0; // reset latency metric
      low_latency_ops();
    }
  }
  else if (p.command==0) { // ping request
    pkt.write_packet(0,p.length,p.data); // ping reply
  }
}

/**** Low latency (sub-millisecond) timing section.
 * We need this for maximum accuracy in our encoder counts.
 */

milli_t last_milli=0;
void low_latency_ops() {
  unsigned long micro=micros();
  milli=micro>>10; // approximately == milliseconds

  //Encoder for mining motor
//  encoder_M.read();
//  // robot.sensor.Mspeed=encoder_M.period;
//  robot.sensor.Mcount=encoder_M.count_mono;

  // Read encoders for left and right mining motors.
  encoder_mining_left.read();
  robot.sensor.McountL = encoder_mining_left.count_mono;
  encoder_mining_right.read();
  robot.sensor.McountR = encoder_mining_right.count_mono;

  //Encoder for roll motor
  encoder_R.read();
  robot.sensor.Rcount=encoder_R.count_dir;

  //Encoder stuff for left drive track
  encoder_DL1.read();
  robot.sensor.DL1count=encoder_DL1.count_dir;

  encoder_DL2.read();
  robot.sensor.DL2count=encoder_DL2.count_dir;

  //Encoder stuff for right drive track
  encoder_DR1.read();
  robot.sensor.DR1count=encoder_DR1.count_dir;

  encoder_DR2.read();
  robot.sensor.DR2count=encoder_DR2.count_dir;
  
  limit_top.read();
  limit_bottom.read();

  // Update latency counter
  unsigned int latency=milli-last_milli;
  if (latency>=1024) latency=1023;
  if (robot.sensor.latency<latency) robot.sensor.latency=latency;
  last_milli=milli;
}


}; // end namespace aurora

void setup()
{
  aurora::PCport.begin(57600); // Control connection to PC via USB

  // Our ONE debug LED!
  //pinMode(13,OUTPUT);
  //digitalWrite(13,LOW);

  // BTS Enable Pin (Controls all pins)
  pinMode(aurora::bts_enable_pin,OUTPUT);
  digitalWrite(aurora::bts_enable_pin,HIGH);

}


milli_t milli;
milli_t next_milli_send=0;
void loop()
{
  aurora::low_latency_ops();

  A_packet p;
  if (aurora::PC.read_packet(p)) aurora::handle_packet(aurora::PC.pkt,p);
  if (!(aurora::PC.is_connected)) aurora::robot.power.stop(); // disconnected?

  if (milli-next_milli_send>=5)
  { // Send commands to motors
    aurora::send_motors();
    next_milli_send=milli; // send_motors every 5ms
  }
}
