/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

This is a simple program to act as the controller process for testing purposes.
It only prints the current system time to the console after doing a relatively
intensive computation to simulate time passing and processer load
-----------------------------------------------------------------------------*/

#include <assert.h>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>

#include <Moby/Simulator.h>
#include <Moby/RCArticulatedBody.h>

#include "ActuatorMessage.h"

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
// Control Functions
//-----------------------------------------------------------------------------

/// Controls the pendulum
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
void control( DynamicBodyPtr pendulum, Real time, void* data ) {

  control_PD( dynamic_pointer_cast<RCArticulatedBody>(pendulum), time );
}

//-----------------------------------------------------------------------------

/// The main control loop for standalone controller
void control( ActuatorMessage& msg ) {

  const Real Kp = 1.0;
  const Real Kv = 0.1;

  Real time = msg.state.time;
  Real measured_position = msg.state.position;
  Real measured_velocity = msg.state.velocity;

  Real desired_position = position_trajectory( time, 0.5, 0.0, PI );
  Real desired_velocity = velocity_trajectory( time, 0.5, 0.0, PI );

  msg.command.torque = Kp * ( desired_position - measured_position ) + Kv * ( desired_velocity - measured_velocity );
}

//-----------------------------------------------------------------------------
// Moby Plugin Interface
//-----------------------------------------------------------------------------

// plugin must be "extern C"
extern "C" {

/**
    Interface to compile as a Moby Plugin
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

  // setup the controller
  pendulum->controller = control;

}

} // end extern C

//-----------------------------------------------------------------------------
// Controller Entry Point
//-----------------------------------------------------------------------------

/**
    Interface to compile as a Standalone Application
*/
int main( int argc, char* argv[] ) {

    ActuatorMessageBuffer amsgbuffer = ActuatorMessageBuffer( 8811, getpid( ) );
    amsgbuffer.open();   // TODO : sanity/safety checking

    //printf( "(controller::initialized)\n" );

    while( 1 ) {
        //printf( "(controller) Reading Command\n" );

        ActuatorMessage msg = amsgbuffer.read( );

        //printf( "(controller) Generating Command\n" );
        control( msg );
        //printf( "(controller) Writing Command\n" );

        amsgbuffer.write( msg );
    }

    amsgbuffer.close( );  // TODO : figure out a way to force a clean up

    return 0;
}
