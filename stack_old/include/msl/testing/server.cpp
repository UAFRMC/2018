#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include "../socket.hpp"
#include "../time.hpp"
#include <vector>

int main()
{
	msl::tcp_socket_t s("0.0.0.0:8080<0.0.0.0:0");
	s.open();

	if(!s.good())
	{
		std::cout<<":("<<std::endl;
		return 0;
	}

	std::cout<<":)\t"<<s.address()<<std::endl;

	std::vector<std::pair<msl::socket_t,std::string>> clients;

	while(s.good())
	{
		auto client=s.accept();

		if(client.good())
		{
			std::cout<<"new client\t"<<client.address()<<std::endl;
			clients.push_back({client,""});
		}

		for(auto ii=std::begin(clients);ii!=std::end(clients);)
		{
			char temp;

			while(ii->first.available()>0&&ii->first.read(&temp,1)==1)
			{
				if(temp=='\n')
				{
					std::cout<<"client said \""<<ii->second<<"\"."<<std::endl;
					ii->first.write("\"You said \""+ii->second+"\".\n");
					ii->second="";
				}
				else
				{
					ii->second+=temp;
				}
			}

			if(!ii->first.good())
			{
				std::cout<<"client disconnected\t"<<ii->first.address()<<std::endl;
				ii=clients.erase(ii);
			}
			else
			{
				++ii;
			}
		}

		msl::delay_ms(1);
	}

	std::cout<<"server shutdown..."<<std::endl;

	return 0;
}