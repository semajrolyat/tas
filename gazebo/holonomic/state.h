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
  // Member Variables
  //---------------------------------------------------------------------------
private:
  std::vector<double> _values;
 
  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
public:
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
  std::vector<double> as_vector( void ) const {
    std::vector<double> v( size() );
    for( unsigned i = 0; i < size(); i++ ) {
      v[i] = _values[i];
    }
    return v;
  }

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

/// Class encapsulating the state of the predator and the prey
/**
 * The first 13 dimensions are the predator state. The next 13 dimensions are
 * the prey state.
 */ 
class pp_state_c {
  //---------------------------------------------------------------------------
  // Member Variables
  //---------------------------------------------------------------------------
private:
  std::vector<double> _values;

  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
public:
  pp_state_c( void ) {
    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = 0.0;
    _values[6] = _values[19] = 1.0; // Normalize w coordinate
  }

  //---------------------------------------------------------------------------
  pp_state_c( const pp_state_c& state ) { 
    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = state._values[i];
  }

  //---------------------------------------------------------------------------
  pp_state_c( const ship_state_c& pred, const ship_state_c& prey ) { 
    _values.resize( size() );
    for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
      _values[i] = pred.value(i);
      _values[ i + ship_state_c::size() ] = prey.value(i);
    }
  }

  //---------------------------------------------------------------------------
  pp_state_c( const std::vector<double>& pred, const std::vector<double>& prey ) { 
    _values.resize( size() );
    for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
      _values[i] = pred[i];
      _values[ i + ship_state_c::size() ] = prey[i];
    }
  }

  //---------------------------------------------------------------------------
  pp_state_c( std::vector<double>& x ) { 
    assert( x.size() == size() );
    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = x[i];
  }

  //---------------------------------------------------------------------------
  pp_state_c( std::valarray<double>& x ) { 
    assert( x.size() == size() );
    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = x[i];
  }

  //---------------------------------------------------------------------------
  // this is disabled b/c it would take too many arguments
  /*
  pp_state_c( gazebo::math::Vector3 x, gazebo::math::Quaternion theta, gazebo::math::Vector3 dx, gazebo::math::Vector3 dtheta ) { 
    _values.resize( size() );
    position( x );
    rotation( theta );
    dposition( dx );
    drotation( dtheta );
  }
  */

  //---------------------------------------------------------------------------
  pp_state_c( const ompl::base::StateSpace* space, const ompl::base::State* x ) { 
    //assert( x.size() == size() );

    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = *space->getValueAddressAtIndex(x, i);
  }

  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~pp_state_c( void ) {}

  //---------------------------------------------------------------------------
  //
  //---------------------------------------------------------------------------
  void position_pred( const gazebo::math::Vector3& v ) {
    _values[0] = v.x;
    _values[1] = v.y;
    _values[2] = v.z;
  }

  void position_prey( const gazebo::math::Vector3& v ) {
    _values[13] = v.x;
    _values[14] = v.y;
    _values[15] = v.z;
  }

  //---------------------------------------------------------------------------
  gazebo::math::Vector3 position_pred( void ) const {
    return gazebo::math::Vector3( _values[0], _values[1], _values[2] );
  }

  gazebo::math::Vector3 position_prey( void ) const {
    return gazebo::math::Vector3( _values[13], _values[14], _values[15] );
  }

  //---------------------------------------------------------------------------
  void rotation_pred( const gazebo::math::Quaternion& q ) {
    _values[3] = q.x;
    _values[4] = q.y;
    _values[5] = q.z;
    _values[6] = q.w;
  }

  void rotation_prey( const gazebo::math::Quaternion& q ) {
    _values[16] = q.x;
    _values[17] = q.y;
    _values[18] = q.z;
    _values[19] = q.w;
  }

  //---------------------------------------------------------------------------
  gazebo::math::Quaternion rotation_pred( void ) const {
    return gazebo::math::Quaternion( _values[6], _values[3], _values[4], _values[5] );
  }

  gazebo::math::Quaternion rotation_prey( void ) const {
    return gazebo::math::Quaternion( _values[19], _values[16], _values[17], _values[18] );
  }

  //---------------------------------------------------------------------------
  void dposition_pred( const gazebo::math::Vector3& v ) {
    _values[7] = v.x;
    _values[8] = v.y;
    _values[9] = v.z;
  }

  void dposition_prey( const gazebo::math::Vector3& v ) {
    _values[20] = v.x;
    _values[21] = v.y;
    _values[22] = v.z;
  }

  //---------------------------------------------------------------------------
  gazebo::math::Vector3 dposition_pred( void ) const {
    return gazebo::math::Vector3( _values[7], _values[8], _values[9] );
  }

  gazebo::math::Vector3 dposition_prey( void ) const {
    return gazebo::math::Vector3( _values[20], _values[21], _values[22] );
  }

  //---------------------------------------------------------------------------
  void drotation_pred( const gazebo::math::Vector3& v ) {
    _values[10] = v.x;
    _values[11] = v.y;
    _values[12] = v.z;
  }

  void drotation_prey( const gazebo::math::Vector3& v ) {
    _values[23] = v.x;
    _values[24] = v.y;
    _values[25] = v.z;
  }


  //---------------------------------------------------------------------------
  gazebo::math::Vector3 drotation_pred( void ) const {
    return gazebo::math::Vector3( _values[10], _values[11], _values[12] );
  }

  gazebo::math::Vector3 drotation_prey( void ) const {
    return gazebo::math::Vector3( _values[23], _values[24], _values[25] );
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
    return 13*2;
  }

  //---------------------------------------------------------------------------
  std::vector<double> as_vector( void ) const {
    std::vector<double> q( size() );
    for( unsigned i = 0; i < size(); i++ ) {
      q[i] = _values[i];
    }
    return q;
  }

  //---------------------------------------------------------------------------
  void write_ompl_state( ompl::base::StateSpace *statespace, ompl::base::State *state ) {
    for( unsigned i = 0; i < size(); i++ )
      *statespace->getValueAddressAtIndex(state, i) = _values[i];
  }

  //---------------------------------------------------------------------------
  // Get the predator state components as a vector
  std::vector<double> pred_vector( void ) const {
    std::vector<double> q( ship_state_c::size() );
    for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
      q[i] = _values[i];
    }
    return q;
  }

  //---------------------------------------------------------------------------
  // Get the prey state components as a vector
  std::vector<double> prey_vector( void ) const {
    std::vector<double> q( ship_state_c::size() );
    for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
      q[i] = _values[ i + ship_state_c::size() ];
    }
    return q;
  }

  //---------------------------------------------------------------------------
  // Set the predator state components from a vector
  void pred_vector( const std::vector<double>& q ) {
    assert( q.size() == ship_state_c::size() );
    for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
      _values[i] = q[i];
    }
  }

  //---------------------------------------------------------------------------
  // Set the prey state components from a vector
  void prey_vector( const std::vector<double> q ) {
    assert( q.size() == ship_state_c::size() );
    for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
      _values[ i + ship_state_c::size() ] = q[i];
    }
  }
 
  //---------------------------------------------------------------------------
  // Output
  //---------------------------------------------------------------------------
public:
  static std::string header( void ) {
    return "t, pred pos[x, y, z], pred rot[x, y, z, w], pred dpos[x, y, z], pred drot[x, y, z], prey pos[x, y, z], prey rot[x, y, z, w], prey dpos[x, y, z], prey drot[x, y, z]";
  }

  friend std::ostream& operator<<(std::ostream& ostr, const pp_state_c& state);
  friend std::ostream& operator<<(std::ostream& ostr, const std::pair<double,pp_state_c>& state);
};

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const pp_state_c& state) {
  return ostr << "predator: " << state.value(0) << "," << state.value(1) << "," << state.value(2) << "," 
              << state.value(3) << "," << state.value(4) << "," << state.value(5) << "," << state.value(6) << "," 
              << state.value(7) << "," << state.value(8) << "," << state.value(9) << "," 
              << state.value(10) << "," << state.value(11) << "," << state.value(12) << "prey: " << state.value(13) << "," << state.value(14) << "," << state.value(15) << "," << state.value(16) << "," << state.value(17) << "," << state.value(18) << "," << state.value(19) << ","  << state.value(20) << "," << state.value(21) << "," << state.value(22) << "," << state.value(23) << "," << state.value(24) << "," << state.value(25);
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const std::pair<double,pp_state_c>& state) {
  return ostr << "t=" << state.first << " " << state.second;
}

//-----------------------------------------------------------------------------
// Note for the below arithmetic operations, need to normalize the quaternion after the computation
pp_state_c operator+( pp_state_c& a, pp_state_c& b ) {

  pp_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) += b(i);
  return state;
}

//-----------------------------------------------------------------------------
pp_state_c operator-( pp_state_c& a, pp_state_c& b ) {

  pp_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) -= b(i);
  return state;
}

//-----------------------------------------------------------------------------
pp_state_c operator*( const double& c, pp_state_c& a ) {
  pp_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) *= c;
  return state;
}

//-----------------------------------------------------------------------------
pp_state_c operator*( pp_state_c& a, const double& c ) {
  pp_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) *= c;
  return state;
}

//-----------------------------------------------------------------------------
pp_state_c operator/( pp_state_c& a, const double& c ) {
  const double EPSILON = 1e-16;
  assert( fabs(c) > EPSILON );

  pp_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) /= c;
  return state;
}


//------------------------------------------------------------------------------
// Auditing 
//------------------------------------------------------------------------------
/// Create an audit file and write header line for state data
bool write_state_audit_header( const std::string& filename ) {
  std::fstream fs;
  fs.open( filename.c_str(), std::fstream::out );
  if( !fs.is_open() ) return false;
  fs << ship_state_c::header() << std::endl;
  fs.close();
  return true;
}

//------------------------------------------------------------------------------
/// Write a single state data element to an audit file (includes time)
bool write_audit_datum( const std::string& filename, const std::pair<double,ship_state_c>& state ) {
  std::fstream fs;
  fs.open( filename.c_str(), std::fstream::out | std::fstream::app );
  if( !fs.is_open() ) return false;
  fs << state << std::endl;
  fs.close();
  return true;
}

//------------------------------------------------------------------------------
/// Write an entire state data set to an audit file
bool write_audit_data( const std::string& filename, const ship_state_list_t& list ) {
  for( unsigned i = 0; i < list.size(); i++ )
    if( !write_audit_datum( filename, list[i] ) ) return false;
  return true;
}

//-----------------------------------------------------------------------------

#endif // _GAZEBO_SHIP_STATE_H_

