#ifndef _AABB_H_
#define _AABB_H_

#include <vector>
#include <iostream>
#include <Ravelin/Origin3d.h>
#include <Ravelin/Vector3d.h>

//-----------------------------------------------------------------------------

class aabb_c;
typedef std::vector<aabb_c> aabb_list_t;

//-----------------------------------------------------------------------------
/// Class encapsulating Axis-Aligned bounding boxes
class aabb_c {
public:
  Ravelin::Vector3d center;
  Ravelin::Vector3d extens;

  //---------------------------------------------------------------------------
  aabb_c( void ) : center( 0.0, 0.0, 0.0 ), extens( 0.0, 0.0, 0.0 ) {
    
  }

  //---------------------------------------------------------------------------
  aabb_c( const Ravelin::Vector3d& _center, const Ravelin::Vector3d& _extens ) :
    center( _center ), 
    extens( _extens ) 
  {

  }
  //---------------------------------------------------------------------------
  aabb_c( const Ravelin::Origin3d& _center, const Ravelin::Origin3d& _extens ) :
    center( Ravelin::Vector3d( _center[0], _center[1], _center[2] ) ), 
    extens( Ravelin::Vector3d( _extens[0], _extens[1], _extens[2] ) ) 
  {

  }
 
  //---------------------------------------------------------------------------
  virtual ~aabb_c( void ) { }

  //---------------------------------------------------------------------------
  static bool intersects( const aabb_c& a, const aabb_c& b ) {
    Ravelin::Vector3d p = a.center - b.center;

    return fabs( p[0] ) <= ( a.extens[0] + b.extens[0] ) &&
           fabs( p[1] ) <= ( a.extens[1] + b.extens[1] ) &&
           fabs( p[2] ) <= ( a.extens[2] + b.extens[2] );
  }

  //---------------------------------------------------------------------------
  static bool inside( const Ravelin::Vector3d& point, const aabb_c& box ) {
    Ravelin::Vector3d p = point - box.center;

    const double EPSILON = 1e-5;
    return fabs( p[0] ) <= box.extens[0] + EPSILON &&
           fabs( p[1] ) <= box.extens[1] + EPSILON &&
           fabs( p[2] ) <= box.extens[2] + EPSILON;
  }
  //---------------------------------------------------------------------------
  //friend std::ostream& operator<<(std::ostream& ostr, const aabb_c& bb);

};
/*
//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const aabb_c& bb) {
  return ostr << "center[" << bb.center.x << "," << bb.center.y << "," << bb.center.z << "]," 
              << "extens[" << bb.extens.x << "," << bb.extens.y << "," << bb.extens.z << "]";
}
*/
//-----------------------------------------------------------------------------

#endif // _AABB_H_

