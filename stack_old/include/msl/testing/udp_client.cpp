#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include "../socket.hpp"
#include "../time.hpp"

int main()
{
	const size_t buffer_size=200;
	const char filler='b';
	//msl::udp_socket_t c("127.0.0.1:8080>127.0.0.1:8081",buffer_size);
	msl::udp_socket_t c("127.0.0.1:8081>127.0.0.1:8080",buffer_size);
	c.open();

	if(!c.good())
	{
		std::cout<<":("<<std::endl;
		return 0;
	}

	std::cout<<":)\t"<<c.address()<<std::endl;

	auto timer=msl::millis()+1000;


	while(c.good())
	{
		if(msl::millis()>=timer)
		{
			std::cout<<"sending packet!"<<std::endl;
			std::string data;
			for(size_t ii=0;ii<buffer_size;++ii)
				data+=filler;
			c.write(data);
			timer=msl::millis()+1000;
		}

		char temp[buffer_size];

		while(c.available()>0&&c.read(temp,buffer_size)==buffer_size)
			std::cout<<"received packet!\t"<<temp[0]<<std::endl;

		msl::delay_ms(1);
	}

	std::cout<<"disconnected from server..."<<std::endl;

	return 0;
}
