#ifndef _COMMAND_H_
#define _COMMAND_H_

//-----------------------------------------------------------------------------

#include <vector>
#include <string>
#include <iosfwd>
#include <iostream>
#include <fstream>

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
std::ostream& operator<<( std::ostream& o, const ship_command_msg_t& cmd );

//-----------------------------------------------------------------------------
class ship_command_c;
typedef std::vector< std::pair<double,ship_command_c> > ship_command_list_t;

//-----------------------------------------------------------------------------
/// Class encapsulating Commands used by a ship
class ship_command_c {

  //---------------------------------------------------------------------------
  // Member Variables
  //---------------------------------------------------------------------------
private:
  std::vector<double> _values;
  double duration; 

  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
public:
  //---------------------------------------------------------------------------
  ship_command_c( void ); 
  ship_command_c( const ship_command_c& command );
  ship_command_c( std::vector<double>& x );
  ship_command_c( const ship_command_msg_t& msg );
  ship_command_c( const double* x );
  ship_command_c( Ravelin::Vector3d F, Ravelin::Vector3d tau ); 
  ship_command_c( const ompl::control::Control* u );
 
  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~ship_command_c( void );

  //---------------------------------------------------------------------------
  //
  //---------------------------------------------------------------------------
  void force( const Ravelin::Vector3d& v );
  Ravelin::Vector3d force( void ) const; 

  void torque( const Ravelin::Vector3d& v );
  Ravelin::Vector3d torque( void ) const;

  double& operator()( const unsigned& i );
  double value( const unsigned& i ) const;

  static unsigned size( void ); 

  std::vector<double> as_vector( void ) const;

  ship_command_msg_t as_msg( void ) const;


  //---------------------------------------------------------------------------
  // Output
  //---------------------------------------------------------------------------
public: 
  static std::string header( void );

  friend std::ostream& operator<<(std::ostream& ostr, const std::pair<double,ship_command_c>& cmd);
  friend std::ostream& operator<<(std::ostream& ostr, const ship_command_c& cmd);
};

//-----------------------------------------------------------------------------
ship_command_c operator+( ship_command_c& a, ship_command_c& b );
//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const ship_command_c& cmd);
//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const std::pair<double,ship_command_c>& cmd);

/*
//-----------------------------------------------------------------------------
// Auditing
//------------------------------------------------------------------------------
/// Create the audit file and write header line for command data
bool write_command_audit_header( const std::string& filename );

//------------------------------------------------------------------------------
/// Write a single command data element to an audit file (includes time)
bool write_audit_datum( const std::string& filename, const std::pair<double,ship_command_c>& cmd );

//------------------------------------------------------------------------------
/// Write an entire command data set to an audit file
bool write_audit_data( const std::string& filename, const ship_command_list_t& list );
//------------------------------------------------------------------------------
*/
#endif // _COMMAND_H_

