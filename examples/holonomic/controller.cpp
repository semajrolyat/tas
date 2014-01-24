/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

controller.cpp
-----------------------------------------------------------------------------*/

#include "controller.h"

#include <Ravelin/Quatd.h>
#include <Ravelin/Vector3d.h>

//-----------------------------------------------------------------------------

using namespace Moby;
using boost::shared_ptr;
using boost::dynamic_pointer_cast;

//-----------------------------------------------------------------------------


#if defined(MOBY_DYNAMICS)
//-----------------------------------------------------------------------------
// Standalone Controller
//----------------------------------------------------------------------------

/*
// error logging facilities
log_c error_log;
char strbuffer[256];

//-----------------------------------------------------------------------------

// shared buffer for actuator messages
sharedbuffer_c amsgbuffer;

//-----------------------------------------------------------------------------

cpuinfo_c cpuinfo;
unsigned long long cpu_speed_hz;

//-----------------------------------------------------------------------------
*/

//-----------------------------------------------------------------------------
controller_c::controller_c( void ) {
  
}

//-----------------------------------------------------------------------------
controller_c::controller_c( char* argv[] ) {
//  const unsigned X = 0, Y = 1, Z = 2;

  std::string name = argv[0];

  DT = atof(argv[1]);

  self = ship_p( new ship_c( argv[2] ) );
  adversary = ship_p( new ship_c( argv[3] ) );
  space = space_p( new space_c( argv[4] ) );

/*

  int id = atoi(argv[2]);
  // Note: this might be able to be pulled from name instead
  if( id == 1 ) {
    self->PLAYER_TYPE = ship_c::PREDATOR;
    self->ADVERSARY_TYPE = ship_c::PREY;
  } else {
    self->PLAYER_TYPE = ship_c::PREY;
    self->ADVERSARY_TYPE = ship_c::PREDATOR;
  }

  // setup mass
  self->inertial.m = atof(argv[3]);

  // setup moment of inertia
  // Row X
  self->inertial.J(X,X) = atof(argv[4]);  
  self->inertial.J(X,Y) = atof(argv[5]);  
  self->inertial.J(X,Z) = atof(argv[6]);

  // Row Y
  self->inertial.J(Y,X) = atof(argv[7]);   
  self->inertial.J(Y,Y) = atof(argv[8]); 
  self->inertial.J(Y,Z) = atof(argv[9]);

  // Row Z
  self->inertial.J(Z,X) = atof(argv[10]);   
  self->inertial.J(Z,Y) = atof(argv[11]);  
  self->inertial.J(Z,Z) = atof(argv[12]);

  FEEDBACK_GAIN_PROPORTIONAL_POSITION = atof(argv[13]);
  FEEDBACK_GAIN_PROPORTIONAL_POSITION = atof(argv[14]);
  FEEDBACK_GAIN_PROPORTIONAL_POSITION = atof(argv[15]);
  FEEDBACK_GAIN_PROPORTIONAL_POSITION = atof(argv[16]);
*/
}

//-----------------------------------------------------------------------------
controller_c::~controller_c( void ) {
  
}

//-----------------------------------------------------------------------------
void controller_c::execute( void ) {

}

//-----------------------------------------------------------------------------

ship_command_c controller_c::compute_feedback( const ship_state_c& q, ship_command_c& u, const double& Kp_position, const double& Kd_position, const double& Kp_rotation, const double& Kd_rotation ) {

  ship_state_c qx;
  compute_desired_state( q, u, qx, DT );

  // normalize qx's quaternion
  Ravelin::Quatd ex(qx.value(3), qx.value(4), qx.value(5), qx.value(6));
  ex.normalize();
  qx(3) = ex.x;
  qx(4) = ex.y;
  qx(5) = ex.z;
  qx(6) = ex.w;

  // get quaternion from current state
  Ravelin::Quatd e(q.value(3), q.value(4), q.value(5), q.value(6));

  ship_state_c deltaq = qx - q;

  Ravelin::Vector3d pos_err( deltaq.value(0), deltaq.value(1), deltaq.value(2) );
  Ravelin::Vector3d vel_err( deltaq.value(7), deltaq.value(8), deltaq.value(9) );

  Ravelin::Quatd rot_err( deltaq.value(3), deltaq.value(4), deltaq.value(5), deltaq.value(6) );
  Ravelin::Vector3d rotv_err( deltaq.value(10), deltaq.value(11), deltaq.value(12) );

  Ravelin::Vector3d F = Kp_position * pos_err + Kd_position * vel_err;

  // convert differential quaternion to angular velocity
  Ravelin::Vector3d rotp = Ravelin::Quatd::to_omega(e, rot_err);

  // multiply by positional gain
  rotp *= Kp_rotation;

  // compute the derivative error of orientation
  Ravelin::Vector3d rotd = rotv_err;

  // multiply by derivative gain
  rotd *= Kd_rotation;

  // compute the combined control
  Ravelin::Vector3d tau = rotp + rotd;

  // build the command
  ship_command_c u_fb;
  u_fb.force( F );
  u_fb.torque( tau );

  return u_fb;
}

//-----------------------------------------------------------------------------
void controller_c::compute_desired_state( const ship_state_c& q0, const ship_command_c& u, ship_state_c& q, const double& dt ) {

  // use forward dynamics to compute the change in state applying the command causes
  std::vector<double> _u = u.as_vector();
  std::vector<double> _q = q0.as_vector();
  std::vector<double> _dq;
  ship_c::ode( _q, _u, _dq, self );

  // integrate the change in state with the current state to get the desired state
  for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
    _q[i] += _dq[i] * dt;
  }
  // normalize q's quaternion
  Ravelin::Quatd ex( _q[3], _q[4], _q[5], _q[6] );
  ex.normalize();
  _q[3] = ex.x;
  _q[4] = ex.y;
  _q[5] = ex.z;
  _q[6] = ex.w;

  // return the desired state
  q = ship_state_c( _q );
}

//-----------------------------------------------------------------------------
/// Control function for Standalone Controller
error_e controller_c::control( const act_msg_c& input, act_msg_c& output ) {
  ship_command_c u( input.body.command );

  // compute the command if prey.  Predator command came from input via planner
  if( self->PLAYER_TYPE == ship_c::PREY ) {
    pp_state_c state = input.state();

/*    std::vector<double> q0_pred = state.pred_state().as_vector();
    std::vector<double> q0_prey = state.prey_state().as_vector();
    std::vector<double> u_prey;

    double t = input.header.time.seconds;

    ship_c::prey_command(q0_pred, q0_prey, u_prey, adversary, self, t );
*/
    double t = input.header.time.seconds;
    self->plan_for_prey( t, state, u );
  }  

  // treat command_current as ff
  ship_command_c u_ff = u;

  ship_state_c qi;
  if( self->PLAYER_TYPE == ship_c::PREDATOR )
    qi = ship_state_c( input.body.state.predator );
  else
    qi = ship_state_c( input.body.state.prey );

  // compute fb
  ship_command_c u_fb = compute_feedback( qi, u, FEEDBACK_GAIN_PROPORTIONAL_POSITION, FEEDBACK_GAIN_DERIVATIVE_POSITION, FEEDBACK_GAIN_PROPORTIONAL_ROTATION, FEEDBACK_GAIN_DERIVATIVE_ROTATION );

  // sum ff and fb
  u = u_ff + u_fb;

  output = act_msg_c( input );
  output.command( u );

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Send a message to the system to request state for a given time
error_e controller_c::request( const simtime_t& simtime, act_msg_c& state ) {
//error_e controller_c::get_state( const double& time, act_msg_c& state ) {

  char buf = 0;
  act_msg_c req;
  controller_notification_c notification;

  // build the request message
  req.header.type = MSG_REQUEST;
  req.header.time = simtime;
  if( self->PLAYER_TYPE == ship_c::PREDATOR ) {
    req.header.requestor = REQUESTOR_PREDATOR;
  } else {
    req.header.requestor = REQUESTOR_PREY;
  }


  if( msg_buffer.write( req ) != BUFFER_ERR_NONE ) {
    sprintf( err_buffer, "(controller.cpp) request(simtime) failed calling sharedbuffer_c.write(req)\n" );
    error_log.write( err_buffer );
    return ERROR_FAILED;
  }

  notification.type = CONTROLLER_NOTIFICATION_ACTUATOR_EVENT;
  notification.duration = 0.0;
  notification.ts = generate_timestamp( );
  //rdtscll( notification.ts );

  int write_channel, read_channel;
  if( self->PLAYER_TYPE == ship_c::PREDATOR ) {
    write_channel = FD_PREDATOR_TO_COORDINATOR_WRITE_CHANNEL;
    read_channel = FD_PREDATOR_TO_COORDINATOR_READ_CHANNEL;
  } else {
    write_channel = FD_PREY_TO_COORDINATOR_WRITE_CHANNEL;
    read_channel = FD_PREY_TO_COORDINATOR_READ_CHANNEL;
  }

  // send a notification to the coordinator that a message has been published
  if( write( write_channel, &notification, sizeof(controller_notification_c) ) == -1 ) {
    std::string err_msg = "(controller.cpp) request(simtime) failed making system call write(...)";
    error_log.write( error_string_bad_write( err_msg , errno ) );
    return ERROR_FAILED;
  }

  // wait for the reply meaning the state request fulfilled and written to the buffer
  // Note: controller blocks here on read
  if( read( read_channel, &buf, 1 ) == -1 ) {
    std::string err_msg = "(controller.cpp) request(simtime) failed making system call read(...)";
    error_log.write( error_string_bad_read( err_msg , errno ) );
    return ERROR_FAILED;
  }

  // At this point a message was sent from the coordinator.  Attempt to read from buffer
  // read the state out of the buffer
  // Note: will block waiting to acquire mutex
  if( msg_buffer.read( state ) != BUFFER_ERR_NONE ) {
    sprintf( err_buffer, "(controller.cpp) request(simtime) failed calling sharedbuffer_c.read(state)\n" );
    error_log.write( err_buffer );
    return ERROR_FAILED;
  }

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Pubish a command 
//error_e controller_c::publish_command( const act_msg_c& cmd ) {
error_e controller_c::publish( const act_msg_c& command ) {

  controller_notification_c notification;

  // send the command message to the actuator
  // Note: will block waiting to acquire mutex
  if( msg_buffer.write( command ) != BUFFER_ERR_NONE) {
    sprintf( err_buffer, "(controller.cpp) publish(command) failed calling sharedbuffer_c.write(command)\n" );
    error_log.write( err_buffer );
    return ERROR_FAILED;
  }

  notification.type = CONTROLLER_NOTIFICATION_ACTUATOR_EVENT;
  notification.duration = 0.0;
  notification.ts = generate_timestamp( );
  //rdtscll( notification.ts );

  int write_channel;
  if( self->PLAYER_TYPE == ship_c::PREDATOR ) {
    write_channel = FD_PREDATOR_TO_COORDINATOR_WRITE_CHANNEL;
  } else {
    write_channel = FD_PREY_TO_COORDINATOR_WRITE_CHANNEL;
  }

  // send a notification to the coordinator
  if( write( write_channel, &notification, sizeof(controller_notification_c) ) == -1 ) {
    std::string err_msg = "(controller.cpp) publish(command) failed making system call write(...)";
    error_log.write( error_string_bad_write( err_msg , errno ) );
    return ERROR_FAILED;
  }

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
// Fulfills the steps in a cycle
error_e controller_c::activate( const simtime_t& simtime ) {

  act_msg_c state, command;

  // query the dyanmics for the state
  if( request( simtime, state ) != ERROR_NONE ) {
    sprintf( err_buffer, "(controller.cpp) activate() failed calling request(simtime(%f),state)\n", simtime.seconds );
    error_log.write( err_buffer );
    return ERROR_FAILED;
  }

  // compute the command message
  if( control( state, command ) != ERROR_NONE ) {
    sprintf( err_buffer, "(controller.cpp) activate() failed calling control(state,command) for simtime %f\n", simtime.seconds );
    error_log.write( err_buffer );
    return ERROR_FAILED;
  }

  // publish the command message
  command.header.type = MSG_COMMAND;
  if( publish( command ) != ERROR_NONE ) {
    sprintf( err_buffer, "(controller.cpp) activate() failed calling publish(command) for simtime %f\n", simtime.seconds );
    error_log.write( err_buffer );
    return ERROR_FAILED;
  }

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Initialization sequence for a standalone controller
void controller_c::init( void ) {

  // connect to the error log
  // Note: error log is created by the coordinator
  error_log = log_c( FD_ERROR_LOG );
  if( error_log.open( ) != LOG_ERROR_NONE ) {
    // Note:  this is not really necessary.  If coordinator launched properly, this should never happen
    printf( "(controller.cpp) ERROR: init() failed calling log_c.open() on FD_ERROR_LOG\nController Exiting\n" );
    exit( 1 );
  }

  // connect to the shared message buffer BEFORE attempting any IPC
  // Note: the message buffer is created by the coordinator before the controller
  // is launched
  //msg_buffer = sharedbuffer_c( BUFFER_NAME, false );
  msg_buffer = sharedbuffer_c( "/amsgbuffer", false );
  if( msg_buffer.open( ) != BUFFER_ERR_NONE ) {
    sprintf( err_buffer, "(controller.cpp) init() failed calling sharedbuffer_c.open(...,false)\nController Exiting\n" );
    printf( "%s", err_buffer );
    error_log.write( err_buffer );
    error_log.close( );
    exit( 1 );
  }

  simtime.seconds = 0.0;
}

//-----------------------------------------------------------------------------

void controller_c::shutdown( void ) {
  printf( "Controller shutting down\n" );

  msg_buffer.close( );  // TODO : figure out a way to force a clean up
}

//-----------------------------------------------------------------------------
#endif

//-----------------------------------------------------------------------------
