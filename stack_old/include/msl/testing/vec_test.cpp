#include <iostream>
#include "../matrix.hpp"
#include "../vector.hpp"

template<typename T> void print(const msl::vector<T>& vec)
{
	for(auto ii:vec)
		std::cout<<ii<<"\t";

	std::cout<<std::endl;
}

template<typename T> void print(const msl::matrix<T>& mat)
{
	for(auto row:mat)
		print(row);

	std::cout<<std::endl;
}

int main()
{
	msl::vec2d a{-30,40};

	print(normalize(a));

	return 0;
}