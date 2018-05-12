/**
 Aurora Robotics network communication code.
 
  Orion Sky Lawlor, lawlor@alaska.edu, 2014-03-23 (Public Domain)
*/
#ifndef __AURORA_ROBOTICS__NETWORK_H
#define __AURORA_ROBOTICS__NETWORK_H

#include "../osl/socket.h"


/**
 This is the simplest telemetry report from the robot.
 It's designed to minimize storage space, although the UDP header
 occupies 28-odd bytes already, so there's no point in sweating every last bit.
*/
class robot_telemetry {
public:
	byte type; ///< 'h' for ordinary telemetry heartbeat
	byte count; ///< counts 0-255 (to detect lost packets, reordering, etc)
	
	byte state; ///< Robot's current state code.  See state codes in robot.h
	byte ack_state; ///< Copy of last-received state change command.
	robot_status_bits status; ///< Robot's current software status (bitfield).
	robot_sensors_arduino sensor; ///< Robot's current raw sensor values (bitfield).  
	robot_localization loc; ///< Backend's current localization values. 
	robot_power power; ///< Current actuator power values (for debugging only)
	
	robot_telemetry() { type='h'; count=0; state=state_STOP; }
};

/**
 This is the command/piloting data sent to the robot.
*/
class robot_command {
public:
	byte type; // 'c' for command packet
	enum {
		command_STOP=0, ///< set the robot to EMERGENCY STOP mode.
		command_state=1, ///< if true, enter the listed state
		command_power=2 ///< set manual driving commands (and exit autopilot/semiauto)
	};
	byte command;
	byte state; ///< Only valid if command_state is true.  See state codes in autonomy.h
	 
	
	robot_power power;
	
	robot_command() { type='c'; command=command_STOP; state=state_STOP; }
};

/**
 This class does low-level network communication to/from the robot.
*/
class robot_comms {
public:
	enum {robot_UDP_telemetry_port=42874}; // telemetry leaves robot on this UDP port number
	enum {robot_UDP_command_port=42875}; // commands enter robot on this port
	unsigned int send_port, recv_port;

	SOCKET socket; // UDP datagram socket for receiving our data
	
	robot_comms() {
#if AURORA_IS_FRONTEND
		recv_port=robot_UDP_telemetry_port; // frontend: receives telemetry
		send_port=robot_UDP_command_port; // sends commands
#elif AURORA_IS_BACKEND
		recv_port=robot_UDP_command_port; // backend: receives commands
		send_port=robot_UDP_telemetry_port; // sends telemetry
#else
#	error "Must define either AURORA_IS_FRONTEND or AURORA_IS_BACKEND!"
#endif
		last_recv_OK=false;

		socket=skt_datagram(&recv_port,0);
		int broadcastEnable=1;
		if (0!=setsockopt(socket, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable))) skt_call_abort("Unable to get broadcast rights on UDP socket!");
	}
	
	struct sockaddr_in last_recv_ip; // Source IP address for last good incoming UDP packet.
	bool last_recv_OK; // if true, the source address is valid.
	
	/* Send the binary data in this object out via UDP. */
	template <class T>
	void broadcast(const T &t)
	{
		/* http://stackoverflow.com/questions/337422/how-to-udp-broadcast-with-c-in-linux 
			255.255.255.255 is the IP local network broadcast address.
		*/
		static struct sockaddr_in bcast_addr=skt_build_addr(skt_lookup_ip("255.255.255.255"),send_port);
		struct sockaddr_in *dest=&bcast_addr;
		static int sendcount=0;
		if (last_recv_OK && (sendcount++%32)!=0) 
		{ // once we get a broadcast, switch to narrowcast (mostly)
			dest=&last_recv_ip;
			// update destination port number
			((struct sockaddr_in *)dest)->sin_port=htons((short)send_port);
		}
		
		if( sendto(socket, &t, sizeof(t), 0, 
		    (struct sockaddr *)dest, sizeof(struct sockaddr_in)) < 0)
			printf("Warning: no network detected (UDP send fail)");
	}
	
	/* If UDP data is available from the other side, 
		return the number of bytes in the next packet.
		If no data is available, return 0.
	*/
	int available(int timeout_msec=1) const {
		if (skt_select1(socket,timeout_msec)!=1) return 0;
		return recvfrom(socket, 0,0, MSG_DONTWAIT|MSG_PEEK|MSG_TRUNC,
			0,0);
			
	}
	
	/* Receive this data via UDP.  
	*/
	template <class T>
	bool receive(T &t) {
		byte buf[sizeof(T)+100];
		struct sockaddr src_addr; socklen_t src_len=sizeof(src_addr);
		int n=recvfrom(socket, buf, sizeof(buf), 0,
			&src_addr,&src_len);
		if (n==sizeof(T)) {
			memcpy(&t,buf,n);
			memcpy(&last_recv_ip,&src_addr,sizeof(last_recv_ip));
			last_recv_OK=true;
			return true;
		}
		else {
			printf("UDP packet size mismatch: expected %d bytes, got %d bytes\n",
				(int)sizeof(T),n);
			return false;
		}
	}
	
	
};

#endif

