/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

controller.cpp
-----------------------------------------------------------------------------*/

#include <assert.h>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>

#include <Moby/Simulator.h>
#include <Moby/RCArticulatedBody.h>

#include <TAS.h>
#include <ActuatorMessage.h>

//-----------------------------------------------------------------------------

using namespace Moby;
using boost::shared_ptr;
using boost::dynamic_pointer_cast;

//-----------------------------------------------------------------------------

// really contingient on what Moby is using for a Real.  In the lab, it is a double
// but Moby can be compliled to have a Real as a single.
typedef double Real;

#define PI 3.14159265359

//-----------------------------------------------------------------------------
// Trajectory Functions
//-----------------------------------------------------------------------------

/// The position trajectory function.  A cubic
Real position_trajectory( Real t, Real tfinal, Real q0, Real qdes ) {

  if( t > tfinal )
    return qdes;
  return -2 * (qdes - q0) / (tfinal*tfinal*tfinal) * (t*t*t) + 3 * (qdes - q0) / (tfinal*tfinal) * (t*t) + q0;
}

//-----------------------------------------------------------------------------

/// The velocity trajectory function.  The first derivative of the cubic
Real velocity_trajectory( Real t, Real tfinal, Real q0, Real qdes ) {

  if( t > tfinal )
    return 0.0;
  return -6 * (qdes - q0) / (tfinal*tfinal*tfinal) * (t*t) + 6 * (qdes - q0) / (tfinal*tfinal) * (t);
}

//-----------------------------------------------------------------------------
// Moby Control Functions
//-----------------------------------------------------------------------------


/// Controls the pendulum
/// Note: Moby Plugin Code
void control_PD( RCArticulatedBodyPtr pendulum, Real time ) {

  const Real Kp = 1.0;
  const Real Kv = 0.1;

  JointPtr pivot = pendulum->find_joint( "pivot" );
  if( !pivot )
    std::cerr << "Could not find pivot joint\n";

  Real measured_position = pivot->q[0];
  Real measured_velocity = pivot->qd[0];

  Real desired_position = position_trajectory( time, 0.5, 0.0, PI );
  Real desired_velocity = velocity_trajectory( time, 0.5, 0.0, PI );

  Real torque = Kp * ( desired_position - measured_position ) + Kv * ( desired_velocity - measured_velocity );

  VectorN tau( 1 );
  tau[0] = torque;
  pivot->add_force( tau );
}

//-----------------------------------------------------------------------------

/// The main control loop for Moby plugin controller
/// Note: Moby Plugin Code
void control( DynamicBodyPtr pendulum, Real time, void* data ) {

  control_PD( dynamic_pointer_cast<RCArticulatedBody>(pendulum), time );
}

//-----------------------------------------------------------------------------
// Mody Plugin Interface
//-----------------------------------------------------------------------------

// plugin must be "extern C"
extern "C" {

/**
    Interface to compile as a Moby Plugin
/// Note: Moby Plugin Code
*/
void init( void* separator, const std::map<std::string, BasePtr>& read_map, Real time ) {

  if( read_map.find("simulator") == read_map.end() )
    throw std::runtime_error( "controller.cpp:init()- unable to find simulator!" );

  // find the pendulum
  if( read_map.find("pendulum") == read_map.end() )
    throw std::runtime_error( "controller.cpp:init()- unable to find pendulum!" );
  DynamicBodyPtr pendulum = dynamic_pointer_cast<DynamicBody>( read_map.find("pendulum")->second );
  if( !pendulum )
    throw std::runtime_error( "controller.cpp:init()- unable to cast pendulum to type DynamicBody" );

  // setup the control function
  pendulum->controller = control;

}

} // end extern C

//-----------------------------------------------------------------------------
// Standalone Controller
//----------------------------------------------------------------------------

ActuatorMessageBuffer amsgbuffer;

//-----------------------------------------------------------------------------

/// Control function for Standalone Controller
ActuatorMessage control( const ActuatorMessage& msg ) {

  ActuatorMessage reply = ActuatorMessage( msg );

  const Real Kp = 1.0;
  const Real Kv = 0.1;

  Real time = msg.state.time;
  Real measured_position = msg.state.position;
  Real measured_velocity = msg.state.velocity;

  Real desired_position = position_trajectory( time, 0.5, 0.0, PI );
  Real desired_velocity = velocity_trajectory( time, 0.5, 0.0, PI );

  reply.command.torque = Kp * ( desired_position - measured_position ) + Kv * ( desired_velocity - measured_velocity );

  //printf( "(controller::control) " );
  //reply.print();

  return reply;
}

//-----------------------------------------------------------------------------

/// Standalone Controller Entry Point
int main( int argc, char* argv[] ) {

    amsgbuffer = ActuatorMessageBuffer( ACTUATOR_MESSAGE_BUFFER_NAME, ACTUATOR_MESSAGE_BUFFER_MUTEX_NAME );
    amsgbuffer.open( );

    ActuatorMessage msg_request, msg_state, msg_command;

    char buf;

    int cycle = 0;
    Real time = 0.0;

    while( 1 ) {

        if( VERBOSE ) printf( "(controller) requesting state\n" );
        // request the state of the actuator
        msg_request.header.type = MSG_TYPE_STATE_REQUEST;
        msg_request.state.time = time;

        amsgbuffer.write( msg_request );

        buf = 0;
        // send a notification to the coordinator
        write( FD_CONTROLLER_TO_COORDINATOR_WRITE_CHANNEL, &buf, 1 );

        // wait for the reply meaning the state request fulfilled and written to the buffer
        if( read( FD_COORDINATOR_TO_CONTROLLER_READ_CHANNEL, &buf, 1 ) == -1 ) {
            switch( errno ) {
            case EINTR:
                // yet to determine appropriate course of action if this happens
                break;
            default:
                printf( "EXCEPTION: unhandled read error in controller reading from FD_COORDINATOR_TO_CONTROLLER_READ_CHANNEL" );
                break;
            }
        }
        if( VERBOSE ) printf( "(controller) received notification of published state\n" );

        // At this point a message was sent from the coordinator.  Attempt to read from buffer
        // read the state out of the buffer
        msg_state = amsgbuffer.read( );

        if( VERBOSE ) printf( "(controller) received state reply\n" );
        if( VERBOSE ) msg_state.print();

        // compute the command message
        msg_command = control( msg_state );
        msg_command.header.type = MSG_TYPE_COMMAND;

        if( VERBOSE ) printf( "(controller) writing command\n" );
        // send the command message to the actuator
        amsgbuffer.write( msg_command );
        // Note: will block waiting to acquire mutex
	
        // send a notification to the coordinator
        write( FD_CONTROLLER_TO_COORDINATOR_WRITE_CHANNEL, &buf, 1 );

        // consume some time.
        //usleep( 1000 );

        time = (double)cycle / (double)CONTROLLER_FREQUENCY_HERTZ;
        cycle++;
    }

    amsgbuffer.close( );  // TODO : figure out a way to force a clean up

    return 0;
}

//-----------------------------------------------------------------------------
