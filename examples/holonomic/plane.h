#ifndef _PLANE_H_
#define _PLANE_H_

//-----------------------------------------------------------------------------

#include <vector>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <Ravelin/Vector3d.h>

//-----------------------------------------------------------------------------

class plane_c;
typedef std::vector<plane_c> plane_list_t;

//-----------------------------------------------------------------------------

class plane_c {
public:

  Ravelin::Vector3d point;
  Ravelin::Vector3d normal;  

  //---------------------------------------------------------------------------
  plane_c( void ) { 

  }

  //---------------------------------------------------------------------------
  plane_c( Ravelin::Vector3d _point, Ravelin::Vector3d _normal ) { 
    point = _point;
    normal = _normal;
  }

  //---------------------------------------------------------------------------
  plane_c( std::string serial ) {

    char* delim = ",";
    char* s = new char[ serial.length()+1 ];
    strcpy( s, serial.c_str() );

    char* tok = strtok( s, delim );

    int i = 0;
    while( tok != NULL ) {
      i++;
      if( i == 1 ) point[0] = atof( tok );
      if( i == 2 ) point[1] = atof( tok );
      if( i == 3 ) point[2] = atof( tok );
      if( i == 4 ) normal[0] = atof( tok );
      if( i == 5 ) normal[1] = atof( tok );
      if( i == 6 ) normal[2] = atof( tok );

      tok = strtok( NULL, delim );
    }

    delete s;
  }

  //---------------------------------------------------------------------------
  virtual ~plane_c( void ) {

  }

  //---------------------------------------------------------------------------
  std::string serialize( void ) {
    std::stringstream ss;

    char* delim = ",";
  
    ss << point[0] << delim;                   //  1
    ss << point[1] << delim;                   //  2
    ss << point[2] << delim;                   //  3
    ss << normal[0] << delim;                  //  4
    ss << normal[1] << delim;                  //  5
    ss << normal[2] << delim;                  //  6

  }

  //---------------------------------------------------------------------------
  bool ray_intersection( const Ravelin::Vector3d& ray_origin, const Ravelin::Vector3d& ray_direction, double& t, Ravelin::Vector3d& intersection ) {

    const double EPSILON = 1e-5;
    // implicit form of plane
    double a = normal.x();
    double b = normal.y();
    double c = normal.z();
    double d = point.dot(-normal);
    // check for near parallel ray and plane
    double denominator = normal.dot( ray_direction );
    if( fabs(denominator) < EPSILON ) {
      if( fabs(ray_origin.x() * a + ray_origin.y() * b + ray_origin.z() * c + d ) < EPSILON ) {
        // ray origin lies in the plane and near parallel
        t = 0;
        return true;
      } else {
        // no intersection, near parallel but origin outside of plane
        return false;
      }
    }

    // non-parallel, compute intersection
    t = -( a * ray_origin.x() + b * ray_origin.y() + c * ray_origin.z() + d);
    t = t / denominator;
    intersection = ray_origin + t * ray_direction;
    // ray only defined for y>=0
    if( t >= 0 )
      return true;
    // return value indicates accept/reject, but still supplying intersection pt
    return false;
  }
};

//-----------------------------------------------------------------------------

#endif // _PLANE_H_
