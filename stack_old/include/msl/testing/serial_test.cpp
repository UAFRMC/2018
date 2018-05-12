#include <iomanip>
#include <iostream>
#include "../serial.hpp"
#include "../time.hpp"

int main()
{
	while(true)
	{
		auto serials=msl::serial_t::list();
		size_t serial_number=0;

		if(serials.size()<=serial_number)
		{
			std::cout<<"serial "<<serial_number<<" does not exist"<<std::endl;
		}
		else
		{
			msl::serial_t serial(serials[serial_number],57600);
			serial.open();

			if(!serial.good())
			{
				std::cout<<"could not open serial port "<<serials[serial_number]<<std::endl;
			}
			else
			{
				std::cout<<"using serial port "<<serials[serial_number]<<std::endl;
			}

			while(serial.good())
			{
				char b;

				while(serial.available()>0&&serial.read(&b,1)==1)
					std::cout<<b<<std::flush;
			}
		}

		msl::delay_ms(100);
	}

	return 0;
}
