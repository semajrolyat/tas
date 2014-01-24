#ifndef _STATE_H_
#define _STATE_H_

//-----------------------------------------------------------------------------

#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <Ravelin/Vector3d.h>
#include <Ravelin/Quatd.h>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>

//-----------------------------------------------------------------------------
struct ship_state_msg_t {
  double x;
  double y;
  double z;
  double qx;
  double qy;
  double qz;
  double qw;
  double dx;
  double dy;
  double dz;
  double drotx;
  double droty;
  double drotz;
};

//-----------------------------------------------------------------------------
std::ostream& operator<<( std::ostream& o, const ship_state_msg_t& state );

//-----------------------------------------------------------------------------
struct pp_state_msg_t {
  ship_state_msg_t predator;
  ship_state_msg_t prey;
};

//-----------------------------------------------------------------------------
std::ostream& operator<<( std::ostream& o, const pp_state_msg_t& state );
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
  ship_state_c( void ); 
  ship_state_c( const ship_state_c& state );
  ship_state_c( const std::vector<double>& x ); 
  ship_state_c( const ship_state_msg_t& msg ); 
  ship_state_c( const Ravelin::Vector3d& x, const Ravelin::Quatd& theta, const Ravelin::Vector3d& dx, const Ravelin::Vector3d& dtheta );
  ship_state_c( const ompl::base::StateSpace* space, const ompl::base::State* x );
  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~ship_state_c( void );

  //---------------------------------------------------------------------------
  //
  //---------------------------------------------------------------------------
  void position( const Ravelin::Vector3d& v );
  Ravelin::Vector3d position( void ) const;
  void rotation( const Ravelin::Quatd& q );
  Ravelin::Quatd rotation( void ) const;
  void dposition( const Ravelin::Vector3d& v );
  Ravelin::Vector3d dposition( void ) const;
  void drotation( const Ravelin::Vector3d& v );
  Ravelin::Vector3d drotation( void ) const;
  double& operator()( const unsigned& i );
  double value( const unsigned& i ) const;
  static unsigned size( void );
  std::vector<double> as_vector( void ) const;
  ship_state_msg_t as_msg( void ) const;

  //---------------------------------------------------------------------------
  // Output
  //---------------------------------------------------------------------------
public:
  static std::string header( void );
 
  friend std::ostream& operator<<(std::ostream& ostr, const ship_state_c& state);
  friend std::ostream& operator<<(std::ostream& ostr, const std::pair<double,ship_state_c>& state);
};

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const ship_state_c& state); 
//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const std::pair<double,ship_state_c>& state); 
//-----------------------------------------------------------------------------
// Note for the below arithmetic operations, need to normalize the quaternion after the computation
ship_state_c operator+( const ship_state_c& a, const ship_state_c& b ); 

//-----------------------------------------------------------------------------
ship_state_c operator-( const ship_state_c& a, const ship_state_c& b );
//-----------------------------------------------------------------------------
ship_state_c operator*( const double& c, const ship_state_c& a ); 
//-----------------------------------------------------------------------------
ship_state_c operator*( const ship_state_c& a, const double& c ); 
//-----------------------------------------------------------------------------
ship_state_c operator/( const ship_state_c& a, const double& c );



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
  pp_state_c( void );
  //---------------------------------------------------------------------------
  pp_state_c( const pp_state_c& state );
  //---------------------------------------------------------------------------
  pp_state_c( const ship_state_c& pred, const ship_state_c& prey );
  //---------------------------------------------------------------------------
  pp_state_c( const std::vector<double>& pred, const std::vector<double>& prey);
  //---------------------------------------------------------------------------
  pp_state_c( const std::vector<double>& x );
  //---------------------------------------------------------------------------
  pp_state_c( const pp_state_msg_t& msg );
  //---------------------------------------------------------------------------
  pp_state_c( const ompl::base::StateSpace* space, const ompl::base::State* x);
  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~pp_state_c( void );

  //---------------------------------------------------------------------------
  //
  //---------------------------------------------------------------------------
  void position_pred( const Ravelin::Vector3d& v );
  void position_pred( const Ravelin::Origin3d& v );
  void position_prey( const Ravelin::Vector3d& v );
  void position_prey( const Ravelin::Origin3d& v );
  //---------------------------------------------------------------------------
  Ravelin::Vector3d position_pred( void ) const;
  Ravelin::Vector3d position_prey( void ) const;
  //---------------------------------------------------------------------------
  void rotation_pred( const Ravelin::Quatd& q );
  void rotation_prey( const Ravelin::Quatd& q );
  //---------------------------------------------------------------------------
  Ravelin::Quatd rotation_pred( void ) const;
  Ravelin::Quatd rotation_prey( void ) const;
  //---------------------------------------------------------------------------
  void dposition_pred( const Ravelin::Vector3d& v );
  void dposition_pred( const Ravelin::Origin3d& v );
  void dposition_prey( const Ravelin::Vector3d& v );
  void dposition_prey( const Ravelin::Origin3d& v );
  //---------------------------------------------------------------------------
  Ravelin::Vector3d dposition_pred( void ) const;
  Ravelin::Vector3d dposition_prey( void ) const;
  //---------------------------------------------------------------------------
  void drotation_pred( const Ravelin::Vector3d& v );
  void drotation_pred( const Ravelin::Origin3d& v );
  void drotation_prey( const Ravelin::Vector3d& v );
  void drotation_prey( const Ravelin::Origin3d& v );

  //---------------------------------------------------------------------------
  Ravelin::Vector3d drotation_pred( void ) const;
  Ravelin::Vector3d drotation_prey( void ) const;
  //---------------------------------------------------------------------------
  double& operator()( const unsigned& i );
  //---------------------------------------------------------------------------
  double value( const unsigned& i ) const;
  //---------------------------------------------------------------------------
  static unsigned size( void );
  //---------------------------------------------------------------------------
  std::vector<double> as_vector( void ) const;
  //---------------------------------------------------------------------------
  pp_state_msg_t as_msg( void ) const;
  //---------------------------------------------------------------------------
  void write_ompl_state( ompl::base::StateSpace *statespace, ompl::base::State *state ) const;
  //---------------------------------------------------------------------------
  ship_state_c pred_state( void ) const;
  //---------------------------------------------------------------------------
  ship_state_c prey_state( void ) const;
  //---------------------------------------------------------------------------
  void pred_state( const ship_state_c& q );
  //---------------------------------------------------------------------------
  void prey_state( const ship_state_c& q );
  //---------------------------------------------------------------------------
  // Get the predator state components as a vector
  std::vector<double> pred_vector( void ) const;
  //---------------------------------------------------------------------------
  // Get the prey state components as a vector
  std::vector<double> prey_vector( void ) const;
  //---------------------------------------------------------------------------
  // Set the predator state components from a vector
  void pred_vector( const std::vector<double>& q );
  //---------------------------------------------------------------------------
  // Set the prey state components from a vector
  void prey_vector( const std::vector<double> q );
  //---------------------------------------------------------------------------
  // Output
  //---------------------------------------------------------------------------
public:
  static std::string header( void );

  friend std::ostream& operator<<(std::ostream& ostr, const pp_state_c& state);
  friend std::ostream& operator<<(std::ostream& ostr, const std::pair<double,pp_state_c>& state);
};

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const pp_state_c& state);
//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const std::pair<double,pp_state_c>& state); 
//-----------------------------------------------------------------------------
// Note for the below arithmetic operations, need to normalize the quaternion after the computation
pp_state_c operator+( const pp_state_c& a, const pp_state_c& b );
//-----------------------------------------------------------------------------
pp_state_c operator-( const pp_state_c& a, const pp_state_c& b );
//-----------------------------------------------------------------------------
pp_state_c operator*( const double& c, const pp_state_c& a );
//-----------------------------------------------------------------------------
pp_state_c operator*( const pp_state_c& a, const double& c );
//-----------------------------------------------------------------------------
pp_state_c operator/( const pp_state_c& a, const double& c );

/*
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
*/
//-----------------------------------------------------------------------------

#endif // _STATE_H_

