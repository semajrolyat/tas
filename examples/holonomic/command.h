#ifndef _COMMAND_H_
#define _COMMAND_H_

//-----------------------------------------------------------------------------

#include <vector>
#include <string>
#include <iostream>

#include <ompl/control/Control.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <Ravelin/Vector3d.h>

//-----------------------------------------------------------------------------
struct ship_command_msg_t {
  double fx;
  double fy;
  double fz;
  double tx;
  double ty;
  double tz;
};

//-----------------------------------------------------------------------------
std::ostream& operator<<( std::ostream& o, const ship_command_msg_t& cmd ) {
  return o << cmd.fx << "," << cmd.fy << "," << cmd.fz << "," 
           << cmd.tx << "," << cmd.ty << "," << cmd.tz;
}

//-----------------------------------------------------------------------------
class ship_command_c;
typedef std::vector< std::pair<double,ship_command_c> > ship_command_list_t;

//-----------------------------------------------------------------------------
/// Class encapsulating Commands used by a ship
class ship_command_c {

  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
public:
  //---------------------------------------------------------------------------
  ship_command_c( void ) {
    _values.resize( size() );

    for( unsigned i = 0; i < size(); i++ )
      _values[i] = 0.0;
  }

  //---------------------------------------------------------------------------
  ship_command_c( const ship_command_c& command ) {
    _values.resize( size() );

    for( unsigned i = 0; i < size(); i++ )
      _values[i] = command._values[i];
  }

  //---------------------------------------------------------------------------
  ship_command_c( std::vector<double>& x ) { 
    assert( x.size() == size() );

    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = x[i];
  }

  //---------------------------------------------------------------------------
  ship_command_c( const ship_command_msg_t& msg ) {
    _values.resize( size() );

    _values[0] = msg.fx;
    _values[1] = msg.fy;
    _values[2] = msg.fz;
    _values[3] = msg.tx;
    _values[4] = msg.ty;
    _values[5] = msg.tz;
  }

  //---------------------------------------------------------------------------
  ship_command_c( const double* x ) { 

    // Note there are pointer safety issues here
    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = x[i];
  }

  //---------------------------------------------------------------------------
  ship_command_c( Ravelin::Vector3d F, Ravelin::Vector3d tau ) { 
    _values.resize( size() );
    force( F );
    torque( tau );
  }

  //---------------------------------------------------------------------------
  ship_command_c( const ompl::control::Control* u ) { 
    //assert( x.size() == size() );

    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = u->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i];
  }

  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~ship_command_c( void ) {}

  //---------------------------------------------------------------------------
  //
  //---------------------------------------------------------------------------
  void force( const Ravelin::Vector3d& v ) {
    _values[0] = v[0];
    _values[1] = v[1];
    _values[2] = v[2];
  }

  //---------------------------------------------------------------------------
  Ravelin::Vector3d force( void ) const {
    return Ravelin::Vector3d( _values[0], _values[1], _values[2] );
  }

  //---------------------------------------------------------------------------
  void torque( const Ravelin::Vector3d& v ) {
    _values[3] = v[0];
    _values[4] = v[1];
    _values[5] = v[2];
  }


  //---------------------------------------------------------------------------
  Ravelin::Vector3d torque( void ) const {
    return Ravelin::Vector3d( _values[3], _values[4], _values[5] );
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
    return 6;
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
  ship_command_msg_t as_msg( void ) const {
    ship_command_msg_t msg;

    msg.fx = _values[0];
    msg.fy = _values[1];
    msg.fz = _values[2];
    msg.tx = _values[3];
    msg.ty = _values[4];
    msg.tz = _values[5];

    return msg;
  }


  //---------------------------------------------------------------------------
  // Member Variables
  //---------------------------------------------------------------------------
private:
  std::vector<double> _values;
  double duration; 

  //---------------------------------------------------------------------------
  // Output
  //---------------------------------------------------------------------------
public: 
  static std::string header( void ) {
    return "t,force[x, y, z], torque[x, y, z]";
  }

  friend std::ostream& operator<<(std::ostream& ostr, const std::pair<double,ship_command_c>& cmd);
  friend std::ostream& operator<<(std::ostream& ostr, const ship_command_c& cmd);
};

//-----------------------------------------------------------------------------
ship_command_c operator+( ship_command_c& a, ship_command_c& b ) {
  ship_command_c command( a );
  for( unsigned i = 0; i < command.size(); i++ )
    command(i) += b(i);
  return command;
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const ship_command_c& cmd) {
  return ostr << cmd.value(0) << "," << cmd.value(1) << "," << cmd.value(2) << "," 
              << cmd.value(3) << "," << cmd.value(4) << "," << cmd.value(5);
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const std::pair<double,ship_command_c>& cmd) {
  return ostr << cmd.first << ","
              << cmd.second.value(0) << "," << cmd.second.value(1) << "," << cmd.second.value(2) << "," 
              << cmd.second.value(3) << "," << cmd.second.value(4) << "," << cmd.second.value(5);
}

//-----------------------------------------------------------------------------
// Auditing
//------------------------------------------------------------------------------
/// Create the audit file and write header line for command data
bool write_command_audit_header( const std::string& filename ) {
  std::fstream fs;
  fs.open( filename.c_str(), std::fstream::out );
  if( !fs.is_open() ) return false;
  fs << ship_command_c::header() << std::endl;
  fs.close();
  return true;
}

//------------------------------------------------------------------------------
/// Write a single command data element to an audit file (includes time)
bool write_audit_datum( const std::string& filename, const std::pair<double,ship_command_c>& cmd ) {
  std::fstream fs;
  fs.open( filename.c_str(), std::fstream::out | std::fstream::app );
  if( !fs.is_open() ) return false;
  fs << cmd << std::endl;
  fs.close();
  return true;
}

//------------------------------------------------------------------------------
/// Write an entire command data set to an audit file
bool write_audit_data( const std::string& filename, const ship_command_list_t& list ) {
  for( unsigned i = 0; i < list.size(); i++ )
    if( !write_audit_datum( filename, list[i] ) ) return false;
  return true;
}

//------------------------------------------------------------------------------

#endif // _COMMAND_H_

