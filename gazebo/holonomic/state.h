#ifndef _GAZEBO_SHIP_STATE_H_
#define _GAZEBO_SHIP_STATE_H_

//-----------------------------------------------------------------------------

#include <vector>
#include <valarray>
#include <string>
#include <iostream>

#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Quaternion.hh>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>

//-----------------------------------------------------------------------------

class ship_state_c;
typedef std::vector< std::pair<double,ship_state_c> > ship_state_list_t;

//-----------------------------------------------------------------------------

/// Class encapsulating the state of a ship
class ship_state_c {
  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
public:
  //---------------------------------------------------------------------------
  ship_state_c( void ) {
    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = 0.0;
    _values[6] = 1.0; // Normalize w coordinate
  }

  //---------------------------------------------------------------------------
  ship_state_c( const ship_state_c& state ) { 
    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = state._values[i];
  }

  //---------------------------------------------------------------------------
  ship_state_c( std::vector<double>& x ) { 
    assert( x.size() == size() );
    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = x[i];
  }

  //---------------------------------------------------------------------------
  ship_state_c( std::valarray<double>& x ) { 
    assert( x.size() == size() );
    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = x[i];
  }

  //---------------------------------------------------------------------------
  ship_state_c( gazebo::math::Vector3 x, gazebo::math::Quaternion theta, gazebo::math::Vector3 dx, gazebo::math::Vector3 dtheta ) { 
    _values.resize( size() );
    position( x );
    rotation( theta );
    dposition( dx );
    drotation( dtheta );
  }

  //---------------------------------------------------------------------------
  ship_state_c( const ompl::base::StateSpace* space, const ompl::base::State* x ) { 
    //assert( x.size() == size() );

    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = *space->getValueAddressAtIndex(x, i);
  }

  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~ship_state_c( void ) {}

  //---------------------------------------------------------------------------
  //
  //---------------------------------------------------------------------------
  void position( const gazebo::math::Vector3& v ) {
    _values[0] = v.x;
    _values[1] = v.y;
    _values[2] = v.z;
  }

  //---------------------------------------------------------------------------
  gazebo::math::Vector3 position( void ) const {
    return gazebo::math::Vector3( _values[0], _values[1], _values[2] );
  }

  //---------------------------------------------------------------------------
  void rotation( const gazebo::math::Quaternion& q ) {
    _values[3] = q.x;
    _values[4] = q.y;
    _values[5] = q.z;
    _values[6] = q.w;
  }

  //---------------------------------------------------------------------------
  gazebo::math::Quaternion rotation( void ) const {
    return gazebo::math::Quaternion( _values[6], _values[3], _values[4], _values[5] );
  }

  //---------------------------------------------------------------------------
  void dposition( const gazebo::math::Vector3& v ) {
    _values[7] = v.x;
    _values[8] = v.y;
    _values[9] = v.z;
  }

  //---------------------------------------------------------------------------
  gazebo::math::Vector3 dposition( void ) const {
    return gazebo::math::Vector3( _values[7], _values[8], _values[9] );
  }

  //---------------------------------------------------------------------------
  void drotation( const gazebo::math::Vector3& v ) {
    _values[10] = v.x;
    _values[11] = v.y;
    _values[12] = v.z;
  }

  //---------------------------------------------------------------------------
  gazebo::math::Vector3 drotation( void ) const {
    return gazebo::math::Vector3( _values[10], _values[11], _values[12] );
  }

  //---------------------------------------------------------------------------
  double& operator()( const unsigned& i ) {
    assert( i < size() );
    return _values[i];
  }

  //---------------------------------------------------------------------------
  double value( const unsigned& i ) const {
    assert( i < size() );
    return _values[i];
  }

  //---------------------------------------------------------------------------
  static unsigned size( void ) {
    return 13;
  }

  //---------------------------------------------------------------------------
  // Member Variables
  //---------------------------------------------------------------------------
private:
  std::vector<double> _values;
 
  //---------------------------------------------------------------------------
  // Output
  //---------------------------------------------------------------------------
public:
  static std::string header( void ) {
    return "t, pos[x, y, z], rot[x, y, z, w], dpos[x, y, z], drot[x, y, z]";
  }

  friend std::ostream& operator<<(std::ostream& ostr, const ship_state_c& state);
  friend std::ostream& operator<<(std::ostream& ostr, const std::pair<double,ship_state_c>& state);
};

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const ship_state_c& state) {
  return ostr << state.value(0) << "," << state.value(1) << "," << state.value(2) << "," 
              << state.value(3) << "," << state.value(4) << "," << state.value(5) << "," << state.value(6) << "," 
              << state.value(7) << "," << state.value(8) << "," << state.value(9) << "," 
              << state.value(10) << "," << state.value(11) << "," << state.value(12);
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const std::pair<double,ship_state_c>& state) {
  return ostr << state.first << ","
              << state.second.value(0) << "," << state.second.value(1) << "," << state.second.value(2) << "," 
              << state.second.value(3) << "," << state.second.value(4) << "," << state.second.value(5) << "," << state.second.value(6) << "," 
              << state.second.value(7) << "," << state.second.value(8) << "," << state.second.value(9) << "," 
              << state.second.value(10) << "," << state.second.value(11) << "," << state.second.value(12);
}

//-----------------------------------------------------------------------------
// Note for the below arithmetic operations, need to normalize the quaternion after the computation
ship_state_c operator+( ship_state_c& a, ship_state_c& b ) {

  ship_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) += b(i);
  return state;
}

//-----------------------------------------------------------------------------
ship_state_c operator-( ship_state_c& a, ship_state_c& b ) {

  ship_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) -= b(i);
  return state;
}

//-----------------------------------------------------------------------------
ship_state_c operator*( const double& c, ship_state_c& a ) {
  ship_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) *= c;
  return state;
}

//-----------------------------------------------------------------------------
ship_state_c operator*( ship_state_c& a, const double& c ) {
  ship_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) *= c;
  return state;
}

//-----------------------------------------------------------------------------
ship_state_c operator/( ship_state_c& a, const double& c ) {
  const double EPSILON = 1e-16;
  assert( fabs(c) > EPSILON );

  ship_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) /= c;
  return state;
}

//-----------------------------------------------------------------------------

#endif // _GAZEBO_SHIP_STATE_H_

