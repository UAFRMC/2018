#include <iostream>
#include "../time.hpp"
#include "../webserver.hpp"

bool client_func(const mg_connection& connection,enum mg_event event)
{
	if(event==MG_AUTH)
		return true;
	if(event!=MG_REQUEST)
		return false;

	std::cout<<connection.uri<<std::endl;
	return false;
}

int main()
{
	std::cout<<sizeof(msl::webserver_t::server_thread_t)<<std::endl;

	while(true)
	{
		msl::webserver_t test(client_func,"0.0.0.0:8080","web");

		test.open();

		if(!test.good())
		{
			std::cout<<":("<<std::endl;
			return 0;
		}

		std::cout<<":)"<<std::endl;

		while(test.good())
			test.close();

		std::cout<<"T_T"<<std::endl;
	}

	return 0;
}