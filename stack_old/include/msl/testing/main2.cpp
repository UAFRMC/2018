#include <cmath>
#include <iostream>
#include <string>
#include "../joystick.hpp"
#include "../time.hpp"

int main()
{
	while(true)
	{
		auto joysticks=msl::joystick::list();
		size_t joystick_number=0;

		if(joysticks.size()<=joystick_number)
		{
			std::cout<<"joystick "<<joystick_number<<" does not exist"<<std::endl;
		}
		else
		{
			msl::joystick joystick(joysticks[joystick_number]);
			joystick.open();

			if(!joystick.good())
			{
				std::cout<<"could not open joystick "<<joystick_number<<std::endl;
			}
			else
			{
				std::cout<<"using joystick number "<<joystick_number<<" with "<<joystick.axis_count()<<
					" axes and "<<joystick.button_count()<<" buttons"<<std::endl;

				while(joystick.good())
				{
					for(int ii=0;ii<4;++ii)
						std::cout<<joystick.axis(ii)<<"\t";
					std::cout<<std::endl;

					msl::delay_ms(1);
				}
			}
		}

		msl::delay_ms(1);
	}

	return 0;
}