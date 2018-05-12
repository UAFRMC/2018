/**
  3D coordinate system transform code.
  
  Modeled directly after the ROS "tf::Transform"

*/
#ifndef __OSL__TRANSFORM_H
#define __OSL__TRANSFORM_H
#include "../osl/vec4.h"

namespace osl {

void vec3_print(const char *heading, vec3 v)
{
  printf("%s: ( %.5f %.5f %.5f )\n", heading, v[0], v[1], v[2]);
}

/** A coordinate frame made from three unit vectors */
class orientation {
public:
  /// Orientation matrix (orthogonal unit vectors)
  vec3 x,y,z;
    
  /// Initialize this orientation to the identity
  orientation() {
    x=vec3(1,0,0);
    y=vec3(0,1,0);
    z=vec3(0,0,1);
  }
  
  
  
  void print(void) const {
    vec3_print("   X",x);
    vec3_print("   Y",y);
    vec3_print("   Z",z);
  }
  
  // Convert local coordinates out into global coordinates
  vec3 global_from_local(vec3 local) const
  {
    return local.x*x + local.y*y + local.z*z;
  }

  // Convert global coordinates down into local coordinates
  vec3 local_from_global(vec3 global) const
  {
    return vec3(dot(global,x), dot(global,y), dot(global,z));
  }
    
  // Apply these incremental rotation angles to this orientation
  //    angle.x rotates about the x axis, etc.
  //    angle vector is in local coordinates, radians, and right handed
  void rotate(vec3 angle)
  {
    z+= angle.y*x - angle.x*y;
    y+=-angle.z*x;
	  x=normalize(cross(y,z));
	  y=normalize(cross(z,x));
	  z=normalize(z);
  }

};




/** A coordinate transform, supporting rotation and translation
  (but not shear or scale) */
class transform {
public:
  vec3 origin; // global coordinates transform origin of local coordinates
  orientation basis; // convert local to global coordinates
  
  transform(vec3 origin_=vec3(0,0,0)) :origin(origin_) {}
  
  // Convert local coordinates out into global coordinates
  vec3 global_from_local(vec3 local) const
  {
    return origin+basis.global_from_local(local);
  }

  // Convert global coordinates down into local coordinates
  vec3 local_from_global(vec3 global) const
  {
    return basis.local_from_global(global-origin);
  }
  
  // Shift this coordinate system relative to this local offset
  void local_translate(vec3 offset) 
  {
    origin += basis.global_from_local(offset);
  }

};

};

#endif



