#include "state.h"


//-----------------------------------------------------------------------------
std::ostream& operator<<( std::ostream& o, const ship_state_msg_t& state ) {
  return o << state.x << "," << state.y << "," << state.z << "," 
           << state.qx <<","<< state.qy <<","<< state.qz <<","<< state.qw <<"," 
           << state.dx << "," << state.dy << "," << state.dz << "," 
           << state.drotx << "," << state.droty << "," << state.drotz;
}

//-----------------------------------------------------------------------------
std::ostream& operator<<( std::ostream& o, const pp_state_msg_t& state ) {
  return o << state.predator << "," << state.prey; 
}

//-----------------------------------------------------------------------------

ship_state_c::ship_state_c( void ) {
  _values.resize( size() );
  for( unsigned i = 0; i < size(); i++ )
    _values[i] = 0.0;
  _values[6] = 1.0; // Normalize w coordinate
}

//-----------------------------------------------------------------------------
ship_state_c::ship_state_c( const ship_state_c& state ) { 
  _values.resize( size() );
  for( unsigned i = 0; i < size(); i++ )
    _values[i] = state._values[i];
}

//-----------------------------------------------------------------------------
ship_state_c::ship_state_c( const std::vector<double>& x ) { 
  assert( x.size() == size() );
  _values.resize( size() );
  for( unsigned i = 0; i < size(); i++ )
    _values[i] = x[i];
}

//-----------------------------------------------------------------------------
ship_state_c::ship_state_c( const ship_state_msg_t& msg ) { 
  _values.resize( size() );

  _values[0] = msg.x;
  _values[1] = msg.y;
  _values[2] = msg.z;
  _values[3] = msg.qx;
  _values[4] = msg.qy;
  _values[5] = msg.qz;
  _values[6] = msg.qw;
  _values[7] = msg.dx;
  _values[8] = msg.dy;
  _values[9] = msg.dz;
  _values[10] = msg.drotx;
  _values[11] = msg.droty;
  _values[12] = msg.drotz;
}

//-----------------------------------------------------------------------------
ship_state_c::ship_state_c( const Ravelin::Vector3d& x, const Ravelin::Quatd& theta, const Ravelin::Vector3d& dx, const Ravelin::Vector3d& dtheta ) { 
  _values.resize( size() );
  position( x );
  rotation( theta );
  dposition( dx );
  drotation( dtheta );
}
//-----------------------------------------------------------------------------
ship_state_c::ship_state_c( const ompl::base::StateSpace* space, const ompl::base::State* x ) { 
  //assert( x.size() == size() );

  _values.resize( size() );
  for( unsigned i = 0; i < size(); i++ )
    _values[i] = *space->getValueAddressAtIndex(x, i);
}

  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
ship_state_c::~ship_state_c( void ) {

}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void ship_state_c::position( const Ravelin::Vector3d& v ) {
  _values[0] = v.x();
  _values[1] = v.y();
  _values[2] = v.z();
}

//-----------------------------------------------------------------------------
Ravelin::Vector3d ship_state_c::position( void ) const {
  return Ravelin::Vector3d( _values[0], _values[1], _values[2] );
}

//-----------------------------------------------------------------------------
void ship_state_c::rotation( const Ravelin::Quatd& q ) {
  _values[3] = q.x;
  _values[4] = q.y;
  _values[5] = q.z;
  _values[6] = q.w;
}

//-----------------------------------------------------------------------------
Ravelin::Quatd ship_state_c::rotation( void ) const {
  return Ravelin::Quatd( _values[3], _values[4], _values[5], _values[6] );
}

//-----------------------------------------------------------------------------
void ship_state_c::dposition( const Ravelin::Vector3d& v ) {
  _values[7] = v.x();
  _values[8] = v.y();
  _values[9] = v.z();
}

//-----------------------------------------------------------------------------
Ravelin::Vector3d ship_state_c::dposition( void ) const {
  return Ravelin::Vector3d( _values[7], _values[8], _values[9] );
}

//-----------------------------------------------------------------------------
void ship_state_c::drotation( const Ravelin::Vector3d& v ) {
  _values[10] = v.x();
  _values[11] = v.y();
  _values[12] = v.z();
}

//-----------------------------------------------------------------------------
Ravelin::Vector3d ship_state_c::drotation( void ) const {
  return Ravelin::Vector3d( _values[10], _values[11], _values[12] );
}

//-----------------------------------------------------------------------------
double& ship_state_c::operator()( const unsigned& i ) {
  assert( i < size() );
  return _values[i];
}

//-----------------------------------------------------------------------------
double ship_state_c::value( const unsigned& i ) const {
  assert( i < size() );
  return _values[i];
}

//-----------------------------------------------------------------------------
unsigned ship_state_c::size( void ) {
  return 13;
}

//-----------------------------------------------------------------------------
std::vector<double> ship_state_c::as_vector( void ) const {
  std::vector<double> v( size() );
  for( unsigned i = 0; i < size(); i++ ) {
    v[i] = _values[i];
  }
  return v;
}

//-----------------------------------------------------------------------------
ship_state_msg_t ship_state_c::as_msg( void ) const {
  ship_state_msg_t msg;

  msg.x = _values[0];
  msg.y = _values[1];
  msg.z = _values[2];
  msg.qx = _values[3];
  msg.qy = _values[4];
  msg.qz = _values[5];
  msg.qw = _values[6];
  msg.dx = _values[7];
  msg.dy = _values[8];
  msg.dz = _values[9];
  msg.drotx = _values[10];
  msg.droty = _values[11];
  msg.drotz = _values[12];

  return msg;
}

//-----------------------------------------------------------------------------
// Output
//-----------------------------------------------------------------------------
std::string ship_state_c::header( void ) {
  return "t, pos[x, y, z], rot[x, y, z, w], dpos[x, y, z], drot[x, y, z]";
}

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
ship_state_c operator+( const ship_state_c& a, const ship_state_c& b ) {

  ship_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) += b.value(i);
  return state;
}

//-----------------------------------------------------------------------------
ship_state_c operator-( const ship_state_c& a, const ship_state_c& b ) {

  ship_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) -= b.value(i);
  return state;
}

//-----------------------------------------------------------------------------
ship_state_c operator*( const double& c, const ship_state_c& a ) {
  ship_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) *= c;
  return state;
}

//-----------------------------------------------------------------------------
ship_state_c operator*( const ship_state_c& a, const double& c ) {
  ship_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) *= c;
  return state;
}

//-----------------------------------------------------------------------------
ship_state_c operator/( const ship_state_c& a, const double& c ) {
  const double EPSILON = 1e-16;
  assert( fabs(c) > EPSILON );

  ship_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) /= c;
  return state;
}

//-----------------------------------------------------------------------------
pp_state_c::pp_state_c( void ) {
  _values.resize( size() );
  for( unsigned i = 0; i < size(); i++ )
    _values[i] = 0.0;
  _values[6] = _values[19] = 1.0; // Normalize w coordinate
}

//-----------------------------------------------------------------------------
pp_state_c::pp_state_c( const pp_state_c& state ) { 
  _values.resize( size() );
  for( unsigned i = 0; i < size(); i++ )
    _values[i] = state._values[i];
}

//-----------------------------------------------------------------------------
pp_state_c::pp_state_c( const ship_state_c& pred, const ship_state_c& prey ) { 
  _values.resize( size() );
  for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
    _values[i] = pred.value(i);
    _values[ i + ship_state_c::size() ] = prey.value(i);
  }
}

//-----------------------------------------------------------------------------
pp_state_c::pp_state_c( const std::vector<double>& pred, const std::vector<double>& prey ) { 
  _values.resize( size() );
  for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
    _values[i] = pred[i];
    _values[ i + ship_state_c::size() ] = prey[i];
  }
}

//-----------------------------------------------------------------------------
pp_state_c::pp_state_c( const std::vector<double>& x ) { 
  assert( x.size() == size() );
  _values.resize( size() );
  for( unsigned i = 0; i < size(); i++ )
    _values[i] = x[i];
}

//-----------------------------------------------------------------------------
pp_state_c::pp_state_c( const pp_state_msg_t& msg ) { 
  _values.resize( size() );

  _values[0] = msg.predator.x;
  _values[1] = msg.predator.y;
  _values[2] = msg.predator.z;
  _values[3] = msg.predator.qx;
  _values[4] = msg.predator.qy;
  _values[5] = msg.predator.qz;
  _values[6] = msg.predator.qw;
  _values[7] = msg.predator.dx;
  _values[8] = msg.predator.dy;
  _values[9] = msg.predator.dz;
  _values[10] = msg.predator.drotx;
  _values[11] = msg.predator.droty;
  _values[12] = msg.predator.drotz;

  _values[13] = msg.prey.x;
  _values[14] = msg.prey.y;
  _values[15] = msg.prey.z;
  _values[16] = msg.prey.qx;
  _values[17] = msg.prey.qy;
  _values[18] = msg.prey.qz;
  _values[19] = msg.prey.qw;
  _values[20] = msg.prey.dx;
  _values[21] = msg.prey.dy;
  _values[22] = msg.prey.dz;
  _values[23] = msg.prey.drotx;
  _values[24] = msg.prey.droty;
  _values[25] = msg.prey.drotz;
}

//-----------------------------------------------------------------------------
pp_state_c::pp_state_c( const ompl::base::StateSpace* space, const ompl::base::State* x ) { 
  //assert( x.size() == size() );

  _values.resize( size() );
  for( unsigned i = 0; i < size(); i++ )
    _values[i] = *space->getValueAddressAtIndex(x, i);
}

//-----------------------------------------------------------------------------
// Destructor
//-----------------------------------------------------------------------------
pp_state_c::~pp_state_c( void ) {

}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void pp_state_c::position_pred( const Ravelin::Vector3d& v ) {
  _values[0] = v.x();
  _values[1] = v.y();
  _values[2] = v.z();
}

//-----------------------------------------------------------------------------
void pp_state_c::position_pred( const Ravelin::Origin3d& v ) {
  _values[0] = v.x();
  _values[1] = v.y();
  _values[2] = v.z();
}

//-----------------------------------------------------------------------------
void pp_state_c::position_prey( const Ravelin::Vector3d& v ) {
    _values[13] = v.x();
    _values[14] = v.y();
    _values[15] = v.z();
  }

//-----------------------------------------------------------------------------
void pp_state_c::position_prey( const Ravelin::Origin3d& v ) {
    _values[13] = v.x();
    _values[14] = v.y();
    _values[15] = v.z();
  }

//-----------------------------------------------------------------------------
Ravelin::Vector3d pp_state_c::position_pred( void ) const {
  return Ravelin::Vector3d( _values[0], _values[1], _values[2] );
}

//-----------------------------------------------------------------------------
Ravelin::Vector3d pp_state_c::position_prey( void ) const {
  return Ravelin::Vector3d( _values[13], _values[14], _values[15] );
}

//-----------------------------------------------------------------------------
void pp_state_c::rotation_pred( const Ravelin::Quatd& q ) {
  _values[3] = q.x;
  _values[4] = q.y;
  _values[5] = q.z;
  _values[6] = q.w;
}

//-----------------------------------------------------------------------------
void pp_state_c::rotation_prey( const Ravelin::Quatd& q ) {
  _values[16] = q.x;
  _values[17] = q.y;
  _values[18] = q.z;
  _values[19] = q.w;
}

//-----------------------------------------------------------------------------
Ravelin::Quatd pp_state_c::rotation_pred( void ) const {
  return Ravelin::Quatd( _values[3], _values[4], _values[5], _values[6] );
}

//-----------------------------------------------------------------------------
Ravelin::Quatd pp_state_c::rotation_prey( void ) const {
  return Ravelin::Quatd( _values[16], _values[17], _values[18], _values[19] );
}

//-----------------------------------------------------------------------------
void pp_state_c::dposition_pred( const Ravelin::Vector3d& v ) {
  _values[7] = v.x();
  _values[8] = v.y();
  _values[9] = v.z();
}

//-----------------------------------------------------------------------------
void pp_state_c::dposition_pred( const Ravelin::Origin3d& v ) {
  _values[7] = v.x();
  _values[8] = v.y();
  _values[9] = v.z();
}

//-----------------------------------------------------------------------------
void pp_state_c::dposition_prey( const Ravelin::Vector3d& v ) {
  _values[20] = v.x();
  _values[21] = v.y();
  _values[22] = v.z();
}

//-----------------------------------------------------------------------------
void pp_state_c::dposition_prey( const Ravelin::Origin3d& v ) {
  _values[20] = v.x();
  _values[21] = v.y();
  _values[22] = v.z();
}

//-----------------------------------------------------------------------------
Ravelin::Vector3d pp_state_c::dposition_pred( void ) const {
  return Ravelin::Vector3d( _values[7], _values[8], _values[9] );
}

//-----------------------------------------------------------------------------
Ravelin::Vector3d pp_state_c::dposition_prey( void ) const {
  return Ravelin::Vector3d( _values[20], _values[21], _values[22] );
}

//-----------------------------------------------------------------------------
void pp_state_c::drotation_pred( const Ravelin::Vector3d& v ) {
  _values[10] = v.x();
  _values[11] = v.y();
  _values[12] = v.z();
}

//-----------------------------------------------------------------------------
void pp_state_c::drotation_pred( const Ravelin::Origin3d& v ) {
  _values[10] = v.x();
  _values[11] = v.y();
  _values[12] = v.z();
}

//-----------------------------------------------------------------------------
void pp_state_c::drotation_prey( const Ravelin::Vector3d& v ) {
  _values[23] = v.x();
  _values[24] = v.y();
  _values[25] = v.z();
}

//-----------------------------------------------------------------------------
void pp_state_c::drotation_prey( const Ravelin::Origin3d& v ) {
  _values[23] = v.x();
  _values[24] = v.y();
  _values[25] = v.z();
}

//-----------------------------------------------------------------------------
Ravelin::Vector3d pp_state_c::drotation_pred( void ) const {
  return Ravelin::Vector3d( _values[10], _values[11], _values[12] );
}

//-----------------------------------------------------------------------------
Ravelin::Vector3d pp_state_c::drotation_prey( void ) const {
  return Ravelin::Vector3d( _values[23], _values[24], _values[25] );
}

//-----------------------------------------------------------------------------
double& pp_state_c::operator()( const unsigned& i ) {
  assert( i < size() );
  return _values[i];
}

//-----------------------------------------------------------------------------
double pp_state_c::value( const unsigned& i ) const {
  assert( i < size() );
  return _values[i];
}

//-----------------------------------------------------------------------------
unsigned pp_state_c::size( void ) {
  return 13*2;
}

//-----------------------------------------------------------------------------
std::vector<double> pp_state_c::as_vector( void ) const {
  std::vector<double> q( size() );
  for( unsigned i = 0; i < size(); i++ ) {
    q[i] = _values[i];
  }
  return q;
}

//-----------------------------------------------------------------------------
pp_state_msg_t pp_state_c::as_msg( void ) const {
  pp_state_msg_t msg;
  msg.predator = pred_state().as_msg();
  msg.predator = prey_state().as_msg();

  return msg;
}

//-----------------------------------------------------------------------------
void pp_state_c::write_ompl_state( ompl::base::StateSpace *statespace, ompl::base::State *state ) const {
  for( unsigned i = 0; i < size(); i++ )
    *statespace->getValueAddressAtIndex(state, i) = _values[i];
}

//-----------------------------------------------------------------------------
ship_state_c pp_state_c::pred_state( void ) const {
  return ship_state_c( pred_vector() );
}

//-----------------------------------------------------------------------------
ship_state_c pp_state_c::prey_state( void ) const {
  return ship_state_c( prey_vector() );
}

//-----------------------------------------------------------------------------
void pp_state_c::pred_state( const ship_state_c& q ) {
  for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
    _values[i] = q.value(i);
  }
}

//-----------------------------------------------------------------------------
void pp_state_c::prey_state( const ship_state_c& q ) {
  for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
    _values[i + ship_state_c::size()] = q.value(i);
  }
}
 
//-----------------------------------------------------------------------------
// Get the predator state components as a vector
std::vector<double> pp_state_c::pred_vector( void ) const {
  std::vector<double> q( ship_state_c::size() );
  for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
    q[i] = _values[i];
  }
  return q;
}

//-----------------------------------------------------------------------------
// Get the prey state components as a vector
std::vector<double> pp_state_c::prey_vector( void ) const {
  std::vector<double> q( ship_state_c::size() );
  for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
    q[i] = _values[ i + ship_state_c::size() ];
  }
  return q;
}

//-----------------------------------------------------------------------------
// Set the predator state components from a vector
void pp_state_c::pred_vector( const std::vector<double>& q ) {
  assert( q.size() == ship_state_c::size() );
  for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
    _values[i] = q[i];
  }
}

//-----------------------------------------------------------------------------
// Set the prey state components from a vector
void pp_state_c::prey_vector( const std::vector<double> q ) {
  assert( q.size() == ship_state_c::size() );
  for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
    _values[ i + ship_state_c::size() ] = q[i];
  }
}
 
//-----------------------------------------------------------------------------
// Output
//-----------------------------------------------------------------------------
std::string pp_state_c::header( void ) {
  return "t, pred pos[x, y, z], pred rot[x, y, z, w], pred dpos[x, y, z], pred drot[x, y, z], prey pos[x, y, z], prey rot[x, y, z, w], prey dpos[x, y, z], prey drot[x, y, z]";
}

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
pp_state_c operator+( const pp_state_c& a, const pp_state_c& b ) {

  pp_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) += b.value(i);
  return state;
}

//-----------------------------------------------------------------------------
pp_state_c operator-( const pp_state_c& a, const pp_state_c& b ) {

  pp_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) -= b.value(i);
  return state;
}

//-----------------------------------------------------------------------------
pp_state_c operator*( const double& c, const pp_state_c& a ) {
  pp_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) *= c;
  return state;
}

//-----------------------------------------------------------------------------
pp_state_c operator*( const pp_state_c& a, const double& c ) {
  pp_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) *= c;
  return state;
}

//-----------------------------------------------------------------------------
pp_state_c operator/( const pp_state_c& a, const double& c ) {
  const double EPSILON = 1e-16;
  assert( fabs(c) > EPSILON );

  pp_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) /= c;
  return state;
}

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


