/**
Robot localization tool: 2D quadric surfaces.

See:
	Garland, Optimal Triangulation and Quadric-Based Surface Simplification

Dr. Orion Lawlor, lawlor@alaska.edu, 2016-05-13 (Public Domain)
*/
#ifndef __OSL_QUADRIC_H
#define __OSL_QUADRIC_H

#include "vec2.h"
#include "mat2.h"

namespace osl {

/**
A quadric surface, i.e. a matrix with 
	v = x^t Q x = x^t A x + 2 b^t x + c
	  = dot(x,Q x)
*/
class quadric {
public:
	/**
	The terms of the A matrix are:
		[	x^2	xy	]
		[	xy	y^2	]
	The matrix is symmetric
	*/
	mat2 A;
	vec2 b;
	float c;
	
	// Create a zero quadric
	quadric() :A(0.0,0.0,0.0,0.0), b(0.0,0.0), c(0.0) {}
	
	/* Create a quadric representing distance from this point. */
	static quadric distance_from(const vec2 &center) {
		quadric Q;
		Q.A=mat2(1.0, 0.0, 0.0, 1.0); // x^2 + y^2 (from origin)
		return Q.translate(center);
	}
	
	/* Create a quadric representing distance from the plane 
	   with normal n and a surface point p */
	quadric(const vec2 &n,const vec2 &p) {
		vec2 N=normalize(n);
		A=mat2(N.x*N.x, N.y*N.x, N.x*N.y, N.y*N.y);
		b=-A*p;
		c=dot(p,-b);
	}
	
	/* Evaluate the quadric at this point */
	float eval(const vec2 &v) const {
		return dot(v,A*v) + 2.0*dot(v,b) + c;
	}
	
	/* Rotate (scale, or skew) this quadric by this matrix M.
	Incoming points v essentially have M pre-applied:
		Q=dot(M*v,A*M*v) + 2.0*dot(M*v,b) + c;
		Q=dot(v,M^T*A*M*v) + 2.0*dot(v,M^T*b) + c;
	*/
	quadric rotate(const mat2 &M) const {
		mat2 Mi=inverse(M); // shift opposite direction
		quadric Q;
		Q.A=Mi.transpose()*A*Mi;
		Q.b=Mi.transpose()*b;
		Q.c=c;
		return Q;
	}
	
	/* Translate this quadric by this vector D.
	Incoming points v essentially have D pre-applied:
		Q=dot(v+D,A*(v+D)) + 2.0*dot(v+D,b) + c;
		Q=dot(v,A*(v+D)) + dot(D,A*(v+D)) + 2.0*dot(v,b)+2.0*dot(D,b) + c;
		Q=dot(v,A*v)+dot(v,A*D) + dot(D,A*v)+dot(D,A*D) + 2.0*dot(v,b)+2.0*dot(D,b) + c;
		
		Q=dot(v,A*v)+dot(v,A*D) + dot(transpose(A)*D,v)+dot(D,A*D) + 2.0*dot(v,b)+2.0*dot(D,b) + c;
	*/
	quadric translate(const vec2 &D) const {
		vec2 Di=-D; // shift opposite direction
		quadric Q;
		Q.A=A;
		Q.b=b+0.5*(A*Di+A.transpose()*Di);
		Q.c=c+dot(Di,A*Di)+2.0*dot(Di,b);
		return Q;
	}
	
	/* Scale back the quadric by this much */
	void scale(float by) {
		A*=by; b*=by; c*=by;
	}
	
	/* Add this quadric to us */
	void add(const quadric &q) {
		A+=q.A; b+=q.b; c+=q.c;
	}
	
	/* Return the determininant of the quadric (error estimate) */
	float det(void) const { return A.det(); }
	
	/* Return the location of the quadric's extremum (maximum or minimum value)  */
	vec2 extremum(void) const {
		// gradient of Q is 2 A v + 2 b, find grad Q == 0 and solve for v.
		return -inverse(A)*b; 
	}
	
};

};



#endif


