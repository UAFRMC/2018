//Author: Aven Bross
//Date: 3/30/2014
//Handles serial connection between backend and arduino

#ifndef __AURORA_ROBOTICS__ROBOT_SERIAL_H
#define __AURORA_ROBOTICS__ROBOT_SERIAL_H

#include "../serial.cpp" // hacky way, throwing undeclared for all serial functions if serial.h is included
#include <sstream>
// #include "robot.h"
#include "display.h"
#include "../cyberalaska/serial_packet.h"

class robot_serial {
public:
	A_packet_formatter<SerialPort> pkt;
	int _timeout;
	void connect();
	void update(robot_base &robot);
	uint32_t McountLdiff, McountRdiff, DL1diff, DR1diff, DL2diff, DR2diff;
	int16_t Rdiff; // Deltas for encoders

	robot_serial() :pkt(Serial) {
		_timeout=100; //< hack, to get connect at startup
		McountLdiff = McountRdiff= DL1diff = DR1diff = DL2diff = DR2diff = 0;
		Rdiff=box_raise_max/2;
	}
};

// Attempt to connect to the arduino
void robot_serial::connect(){
	static int reset_count=0;
	if ((Serial.begin(57600) == -1)) // serial port to Arduino
	{
		if ((++reset_count%400)==0) { // try a resetusb
			int err=system("sudo ./resetusb");
			robotPrintln("Resetusb return code: %d\n",err);
		}
		robotPrintln("Attempting to connect");
		usleep(50*1000); //50ms delay
	}
	else {
		robotPrintln("Opened Arduino port.  Waiting for data.");
		int r=0;
		while (-1==(r=Serial.read())) {}
		robotPrintln("Connected to Arduino.  First byte: %02x",r);
	}
}

void robot_serial::update(robot_base &robot){
	bool got_data=false;

	// Send off power command:
	if (_timeout==0) {
		pkt.write_packet(0x7,sizeof(robot.power),&robot.power);
	}

	// See if robot sends anything back:
	do {
		A_packet p;
		while (-1==pkt.read_packet(p)) { // receive packet data
			got_data=true;
		}
		if (p.valid)
		{
			if (p.command==0)
			{
				robotPrintln("Got echo packet back from robot");
			}
			else if (p.command==0xE)
			{
				robotPrintln("Got ERROR (0xE) packet back from robot (length %d)", p.length);
			}
			else if (p.command==0x3)
			{
				// sensor data
				if (!p.get(robot.sensor))
				{
					robotPrintln("Size mismatch on arduino -> PC sensor packet (expected %d, got %d)",sizeof(robot.sensor),p.length);
				}
				else
				{
					//No longer needed since we added limit switches
					//if(robot.power.mineEncoderReset!=0)
					//{ // user pressed key to zero out rollcount
					//	Rdiff=-robot.sensor.Rcount;
					//}

					// got valid sensor report: arduino is OK
					robot.status.arduino=1;
					robot.sensor.McountL += McountLdiff;
					robot.sensor.McountR += McountRdiff;
					robot.sensor.DL1count += DL1diff;
					robot.sensor.DL2count += DL2diff;
					robot.sensor.DR1count += DR1diff;
					robot.sensor.DR2count += DR2diff;
					robot.sensor.Rcount += Rdiff;
				}
			}
			else
			{ // unknown packet type?!
				robotPrintln("Got unknown packet type 0x%x length %d from robot",p.command,p.length);
			}
		}
		else
		{ /* some sort of serial error? */
			break;
		}
	} while (Serial.available()); // keep reading to clean out buffer

	if(got_data)
	{
		_timeout=0;
	}
	else if(_timeout>5)
	{
		robot.status.arduino=0;
		robotPrintln("Connection Lost");

		// Save old encoder counts, so we don't lose position when Arduino drops
		McountLdiff = robot.sensor.McountL;
		McountRdiff = robot.sensor.McountR;
		DL1diff = robot.sensor.DL1count;
		DL2diff = robot.sensor.DL2count;
		DR1diff = robot.sensor.DR1count;
		DR2diff = robot.sensor.DR2count;
		DR2diff = robot.sensor.DR2count;
		Rdiff = robot.sensor.Rcount;
		connect();
	}
	else{
		_timeout++;
	}

}

#endif
