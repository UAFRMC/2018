#include <iomanip>
#include <iostream>
#include "../joystick.hpp"
#include "../time.hpp"

int main()
{
	while(true)
	{
		auto joysticks=msl::joystick_t::list();
		size_t joystick_number=0;

		if(joysticks.size()<=joystick_number)
		{
			std::cout<<"joystick "<<joystick_number<<" does not exist"<<std::endl;
		}
		else
		{
			msl::joystick_t joystick(joysticks[joystick_number]);
			joystick.open();

			if(!joystick.good())
			{
				std::cout<<"could not open joystick "<<joystick_number<<std::endl;
			}
			else
			{
				std::cout<<"using joystick number "<<joystick_number<<" with "<<joystick.axis_count()<<
					" axes and "<<joystick.button_count()<<" buttons"<<std::endl;
			}

			while(joystick.good())
			{
				for(size_t ii=0;ii<4;++ii)
					std::cout<<std::setw(16)<<joystick.axis(ii);
				std::cout<<std::endl;
			}
		}

		msl::delay_ms(100);
	}

	return 0;
}
