#include <iostream>
#include <stdlib.h>
#include "mat2.h"
#include "quadric.h"
using osl::quadric;
using osl::mat2;

float randfloat() {
	return 1.0+(rand()&0xfff)*(1.0/0xfff);
}
vec2 randvec2(void) {
	return vec2(randfloat(),randfloat());
}
mat2 randmat2(void) {
	return mat2(randvec2(),randvec2());
}
quadric randquadric(void) {
	quadric Q;
	Q.A=randmat2();
	Q.A.x.y=Q.A.y.x; // matrix is symmetric
	Q.b=randvec2();
	Q.c=randfloat();
	return Q;
}
bool same_float(double f1,double f2) {
	double error=fabs(f1-f2);
	double mag=std::max(fabs(f1),fabs(f2));
	double rel_error=(fabs(error)/(1.0+mag));
	bool same=rel_error<0.00001;
	if (!same) std::cout<<"Different floats: "<<f1<<"  vs "<<f2<<" (rel error "<<error<<")\n";
	return same;
}
bool same_vec2(vec2 v1, vec2 v2) {
	return same_float(v1.x,v2.x) && same_float(v1.y,v2.y);
}
bool same_quadric(const quadric &q1,const quadric &q2) {
	for (int test=0;test<40;test++) {
		vec2 v=randvec2();
		if (!same_float(q1.eval(v),q2.eval(v))) return false;
	}
	return true;
}
template <class shifter>
bool same_quadric(const quadric &q1,shifter sh,const quadric &q2) {
	for (int test=0;test<40;test++) {
		vec2 v=randvec2();
		if (!same_float(q1.eval(sh(v)),q2.eval(v))) return false;
	}
	return true;
}

#define testcode(test) if (!test) { std::cout<<"Test failed at repeat "<<repeat<<": "<<#test<<"\n"; exit(1); }

int main() {
	for (int repeat=0;repeat<10;repeat++)
	{
		srand(repeat);
		
		// Random quadrics:
		quadric Q=randquadric();
		testcode(same_quadric(Q,Q));
		
		// Known planes:
		vec2 origin=vec2(2.5,4.3);
		quadric from_origin=quadric::distance_from(origin);
		testcode(same_vec2(origin,from_origin.extremum()));
		
		quadric from_X(vec2(3.0,0.0) /* normal */,  origin);
		for (int test=0;test<50;test++) {
			vec2 v=randvec2();
			double d1=from_X.eval(v);
			double d2=pow(v.x-origin.x,2.0);
			testcode(same_float(d1,d2));
		}
		
		mat2 rotscale=randmat2();
		testcode(same_quadric(Q,[=](vec2 v) {return inverse(rotscale)*v;},Q.rotate(rotscale)));
		
		vec2 shift=randvec2();
		testcode(same_quadric(Q,[=](vec2 v) {return v-shift;},Q.translate(shift)));
	}
	
	std::cout<<"All tests passed\n";
	return 0;
}


