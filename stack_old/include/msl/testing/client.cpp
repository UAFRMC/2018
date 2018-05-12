#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include "../socket.hpp"
#include "../time.hpp"

int main()
{
	msl::tcp_socket_t c("0.0.0.0:0>0.0.0.0:8080");
	c.open();

	if(!c.good())
	{
		std::cout<<":("<<std::endl;
		return 0;
	}

	std::cout<<":)\t"<<c.address()<<std::endl;

	std::string message="";

	auto timer=msl::millis()+1000;


	while(c.good())
	{
		if(msl::millis()>=timer)
		{
			c.write("hello server!\n");
			timer=msl::millis()+1000;
		}

		char temp;

		while(c.available()>0&&c.read(&temp,1)==1)
		{
			if(temp=='\n')
			{
				std::cout<<"server said \""<<message<<"\"."<<std::endl;
				message="";
			}
			else
			{
				message+=temp;
			}
		}

		msl::delay_ms(1);
	}

	std::cout<<"disconnected from server..."<<std::endl;

	return 0;
}