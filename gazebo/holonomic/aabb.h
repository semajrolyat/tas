#ifndef _AABB_H_
#define _AABB_H_

#include <vector>
#include <iostream>
#include <gazebo/math/Vector3.hh>
#include <Ravelin/Origin3d.h>

//-----------------------------------------------------------------------------

class aabb_c;
typedef std::vector<aabb_c> aabb_list_t;

//-----------------------------------------------------------------------------
/// Class encapsulating Axis-Aligned bounding boxes
class aabb_c {
public:
  gazebo::math::Vector3 center;
  gazebo::math::Vector3 extens;

  //---------------------------------------------------------------------------
  aabb_c( void ) : center( 0.0, 0.0, 0.0 ), extens( 0.0, 0.0, 0.0 ) {
    
  }

  //---------------------------------------------------------------------------
  aabb_c( const gazebo::math::Vector3& _center, const gazebo::math::Vector3& _extens ) : 
    center( _center ), 
    extens( _extens ) 
  {
    
  }

  //---------------------------------------------------------------------------
  aabb_c( const Ravelin::Origin3d& _center, const Ravelin::Origin3d& _extens ) {
    center = gazebo::math::Vector3( (double)_center.x(), (double)_center.y(), (double)_center.z() );
    extens = gazebo::math::Vector3( (double)_extens.x(), (double)_extens.y(), (double)_extens.z() );
  }
 
  //---------------------------------------------------------------------------
  virtual ~aabb_c( void ) { }

  //---------------------------------------------------------------------------
  static bool intersects( const aabb_c& a, const aabb_c& b ) {
    gazebo::math::Vector3 p = a.center - b.center;

    return fabs( p.x ) <= ( a.extens.x + b.extens.x) &&
           fabs( p.y ) <= ( a.extens.y + b.extens.y) &&
           fabs( p.z ) <= ( a.extens.z + b.extens.z);
  }

  //---------------------------------------------------------------------------
  static bool inside( const Ravelin::Vector3d& point, const aabb_c& box ) {
    gazebo::math::Vector3 p( point.x(), point.y(), point.z() );
    return inside( p, box );
  }
  //---------------------------------------------------------------------------
  static bool inside( const gazebo::math::Vector3& point, const aabb_c& box ) {
    gazebo::math::Vector3 p = point - box.center;

    return fabs( p.x ) <= box.extens.x &&
           fabs( p.y ) <= box.extens.y &&
           fabs( p.z ) <= box.extens.z;
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

