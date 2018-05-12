#include <iostream>

#include "../file.hpp"

int main()
{
	std::cout<<msl::string_to_file("hello world!","test.txt")<<std::endl;

	std::string data;

	bool worked=msl::file_to_string("test.txt",data);

	std::cout<<worked<<std::endl;

	if(worked)
		std::cout<<data<<std::endl;

	for(auto file:msl::list_files(".."))
		std::cout<<file<<std::endl;

	std::cout<<std::endl;

	for(auto dir:msl::list_directories(".."))
		std::cout<<dir<<std::endl;

	return 0;
}