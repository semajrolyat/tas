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

#include <tas/tas.h>
#include <tas/time.h>
#include <tas/actuator.h>
#include <tas/log.h>
#include <tas/cpu.h>
#include <tas/experiment.h>

//-----------------------------------------------------------------------------

using namespace Moby;
using boost::shared_ptr;
using boost::dynamic_pointer_cast;

//-----------------------------------------------------------------------------

// really dependent on what Moby is using for a Real.  In the lab, it is a double
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
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Standalone Controller
//----------------------------------------------------------------------------

// error logging facilities
log_c error_log;
char strbuffer[256];

//-----------------------------------------------------------------------------

// shared buffer for actuator messages
actuator_msg_buffer_c amsgbuffer;

//-----------------------------------------------------------------------------

cpuinfo_c cpuinfo;
unsigned long long cpu_speed_hz;

//-----------------------------------------------------------------------------

/// Control function for Standalone Controller
error_e control( const actuator_msg_c& input, actuator_msg_c& output ) {

    output = actuator_msg_c( input );

    const Real Kp = 1.0;
    const Real Kv = 0.1;

    Real time = input.state.time;
    Real measured_position = input.state.position;
    Real measured_velocity = input.state.velocity;

    Real desired_position = position_trajectory( time, 0.5, 0.0, PI );
    Real desired_velocity = velocity_trajectory( time, 0.5, 0.0, PI );

    //Real desired_position = position_trajectory( time, 10.0, 0.0, PI );
    //Real desired_velocity = velocity_trajectory( time, 10.0, 0.0, PI );

    output.command.torque = Kp * ( desired_position - measured_position ) + Kv * ( desired_velocity - measured_velocity );

    return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Send a message to the dynamics to publish a state to the actuator message
/// buffer for given time
error_e get_state( const Real& time, actuator_msg_c& state ) {

    char buf = 0;
    actuator_msg_c request;
    controller_notification_c notification;

    // build the request message
    request.header.type = ACTUATOR_MSG_REQUEST;
    request.state.time = time;

    if( amsgbuffer.write( request ) != BUFFER_ERROR_NONE ) {
        sprintf( strbuffer, "(controller.cpp) get_state(time) failed calling actuator_msg_buffer_c.write(request)\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }

    notification.type = CONTROLLER_NOTIFICATION_ACTUATOR_EVENT;
    notification.duration = 0.0;
    rdtscll( notification.ts );

    // send a notification to the coordinator that a message has been published
    if( write( FD_CONTROLLER_TO_COORDINATOR_WRITE_CHANNEL, &notification, sizeof(controller_notification_c) ) == -1 ) {
        std::string err_msg = "(controller.cpp) get_state(time) failed making system call write(...)";
        error_log.write( error_string_bad_write( err_msg , errno ) );
        return ERROR_FAILED;
    }

    // wait for the reply meaning the state request fulfilled and written to the buffer
    // Note: controller blocks here on read
    if( read( FD_COORDINATOR_TO_CONTROLLER_READ_CHANNEL, &buf, 1 ) == -1 ) {
        std::string err_msg = "(controller.cpp) get_state(time) failed making system call read(...)";
        error_log.write( error_string_bad_read( err_msg , errno ) );
        return ERROR_FAILED;
    }

    // At this point a message was sent from the coordinator.  Attempt to read from buffer
    // read the state out of the buffer
    // Note: will block waiting to acquire mutex
    if( amsgbuffer.read( state ) != BUFFER_ERROR_NONE ) {
        sprintf( strbuffer, "(controller.cpp) get_state(time) failed calling actuator_msg_buffer_c.read(state)\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }

    return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Send a command to the actuator message buffer
error_e publish_command( const actuator_msg_c& cmd ) {

    controller_notification_c notification;

    // send the command message to the actuator
    // Note: will block waiting to acquire mutex
    if( amsgbuffer.write( cmd ) != BUFFER_ERROR_NONE) {
        sprintf( strbuffer, "(controller.cpp) publish_command(cmd) failed calling actuator_msg_buffer_c.write(cmd)\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }

    notification.type = CONTROLLER_NOTIFICATION_ACTUATOR_EVENT;
    notification.duration = 0.0;
    rdtscll( notification.ts );

    // send a notification to the coordinator
    if( write( FD_CONTROLLER_TO_COORDINATOR_WRITE_CHANNEL, &notification, sizeof(controller_notification_c) ) == -1 ) {
        std::string err_msg = "(controller.cpp) publish_command(cmd) failed making system call write(...)";
        error_log.write( error_string_bad_write( err_msg , errno ) );
        return ERROR_FAILED;
    }

    return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Fulfill controller need to get initial state information and issue
/// any initial command that might result from initial state
// TODO : add more error handling
error_e query_control_publish( const Real& t ) {

    actuator_msg_c state, command;

    // query the dyanmics for the initial state
    if( get_state( t, state ) != ERROR_NONE ) {
        sprintf( strbuffer, "(controller.cpp) get_initial_state() failed calling get_state(0.0,state)\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }

    // compute the initial command message
    if( control( state, command ) != ERROR_NONE ) {
        sprintf( strbuffer, "(controller.cpp) get_initial_state() failed calling control(state,command)\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }
    command.header.type = ACTUATOR_MSG_COMMAND;

    // publish the initial command message
    if( publish_command( command ) != ERROR_NONE ) {
        sprintf( strbuffer, "(controller.cpp) get_initial_state() failed calling publish_command(command)\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }

    return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Initialization sequence for a standalone controller
void init( void ) {

    // connect to the error log
    // Note: error log is created by the coordinator
    error_log = log_c( FD_ERROR_LOG );
    if( error_log.open( ) != LOG_ERROR_NONE ) {
        // Note:  this is not really necessary.  If coordinator launched properly, this should never happen
        printf( "(controller.cpp) ERROR: main() failed calling log_c.open() on FD_ERROR_LOG\nController Exiting\n" );
        exit( 1 );
    }

    // connect to the shared message buffer BEFORE attempting any IPC
    // Note: the message buffer is created by the coordinator before the controller
    // is launched
    amsgbuffer = actuator_msg_buffer_c( ACTUATOR_MSG_BUFFER_NAME, ACTUATOR_MSG_BUFFER_MUTEX_NAME, false );
    if( amsgbuffer.open( ) != BUFFER_ERROR_NONE ) {
        sprintf( strbuffer, "(controller.cpp) init(argc,argv) failed calling actuator_msg_buffer_c.open(...,false)\nController Exiting\n" );
        printf( "%s", strbuffer );

        error_log.write( strbuffer );
        error_log.close( );
        exit( 1 );
    }

}

//-----------------------------------------------------------------------------

void shutdown( void ) {
    printf( "Controller shutting down\n" );

    amsgbuffer.close( );  // TODO : figure out a way to force a clean up
}

//-----------------------------------------------------------------------------

/// Standalone Controller Entry Point
// TODO : refactor to exit as gracefully as possible
int main( int argc, char* argv[] ) {

    int cycle = 0;
    controller_notification_c notification;
    unsigned long long ts;
    const Real INTERVAL = 1.0 / (double) CONTROLLER_HZ;
    Real t = 0.0;

    // Note: if init fails, the controller bombs out.  Will write a message to
    // console and/or error_log if this happens.
    init( );

    // query and publish initial values
    query_control_publish( 0.0 );

    notification.type = CONTROLLER_NOTIFICATION_SNOOZE;
    notification.duration = INTERVAL;

    // lock into memory to prevent pagefaults
    mlockall( MCL_CURRENT );

    // get the current timestamp
    rdtscll( ts );

    // start the main process loop
    while( 1 ) {

        notification.ts = ts;

        // send a snooze notification
        write( FD_CONTROLLER_TO_COORDINATOR_WRITE_CHANNEL, &notification, sizeof( controller_notification_c ) );

        //READ? to get the controller to block and to ensure it doesn't overrun?  Ideally, the write event
        // causes the coordinator to wake and interrupt this process so there shouldn't be overrun.

        // unsnoozed here, so get a new timestamp
        rdtscll( ts );
        // Note: may have some long term drift due to fp math
        t += INTERVAL;

        query_control_publish( t );

        cycle++;
    }

    munlockall( );

    shutdown( );

    return 0;
}

//-----------------------------------------------------------------------------
