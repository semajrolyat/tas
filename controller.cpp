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

#include <tas.h>
#include <actuator.h>
#include <log.h>
#include <cpu.h>

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

//----------------------------------------------------------------------------

log_c time_log;

//-----------------------------------------------------------------------------

error_e open_time_log( void ) {
    time_log = log_c( "controller_internal_time.dat" );

    if( time_log.open( ) != LOG_ERROR_NONE ) {
        sprintf( strbuffer, "(controller.cpp) open_time_log() failed calling log_c.open() on file controller_internal_time.dat\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }
    return ERROR_NONE;
}

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
error_e get_initial_state( void ) {

    actuator_msg_c state, command;

    // query the dyanmics for the initial state
    if( get_state( 0.0, state ) != ERROR_NONE ) {
        sprintf( strbuffer, "(controller.cpp) get_initial_state() failed calling get_state(0.0,state)\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }

    if( VERBOSE ) printf( "(controller) received initial state\n" );
    if( VERBOSE ) state.print();

    // compute the initial command message
    if( control( state, command ) != ERROR_NONE ) {
        sprintf( strbuffer, "(controller.cpp) get_initial_state() failed calling control(state,command)\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }
    command.header.type = ACTUATOR_MSG_COMMAND;

    // publish the initial command message
    if( VERBOSE ) printf( "(controller) publishing initial command\n" );
    if( publish_command( command ) != ERROR_NONE ) {
        sprintf( strbuffer, "(controller.cpp) get_initial_state() failed calling publish_command(command)\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }

    return ERROR_NONE;
}

//-----------------------------------------------------------------------------

error_e read_cpuinfo( void ) {
    if( cpuinfo.load( ) != CPUINFO_ERROR_NONE ) {
        sprintf( strbuffer, "(controller.cpp) read_cpuinfo() failed calling cpuinfo_c.load()\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }
    printf( "cpuinfo loaded\n" );

    double cpu_speed_mhz;

    for( unsigned int i = 0; i < cpuinfo.cpus.size( ); i++ ) {
        //printf( "processor: %d\n", cpuinfo.cpus.at( i ).processor );
        if( cpuinfo.cpus.at( i ).processor == DEFAULT_PROCESSOR ) {
            cpu_speed_mhz = cpuinfo.cpus.at( i ).mhz;
            // Note: fp conversion will most likely result in inaccuracy
            cpu_speed_hz = (unsigned long long) (cpu_speed_mhz * 1E6);
            // down and dirty fix to fp inaccuracy.
            // TODO : use a much better fix
            if( cpu_speed_hz % 2 == 1 ) cpu_speed_hz++;
            break;
        }
        if( i == cpuinfo.cpus.size( ) - 1 ) {
            sprintf( strbuffer, "(controller.cpp) read_cpuinfo() failed to find default processor in cpus.\n" );
            error_log.write( strbuffer );
            return ERROR_FAILED;
        }
    }
    printf( "cpu speed (MHz): %f\n", cpu_speed_mhz );
    printf( "cpu speed (Hz): %lld\n", cpu_speed_hz );

    unsigned long long cal_hz = calibrate_cycles_per_second( );
    printf( "calibrated cpu speed (Hz): %lld\n", cal_hz );

    return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Query the system for the current time.  Used by polling loop. 
// TODO : refactor with error codes
struct timeval get_process_time( void ) {
    struct rusage ru;
    struct timeval tv;

    // Note: getrusage queries appear to result in a fixed interval (4ms)
    // on Ubuntu 12.04 out of the box.  This 4ms has shown up in rttimers as
    // well.  Researching possible ways to augment the system limitation
    //int result = getrusage( RUSAGE_SELF, &ru );
    //memcpy( &tv, &ru.ru_utime, sizeof(timeval) );

    // so instead use gettimeofday
    gettimeofday( &tv, NULL );

    return tv;
}

//-----------------------------------------------------------------------------
/// Initialization sequence for a standalone controller
void init( void ) {

    // connect to the error log
    // Note: error log is created by the coordinator
    error_log = log_c( FD_ERROR_LOG );
    if( error_log.open( ) != LOG_ERROR_NONE ) {
        // Note:  this is not really necessary
        printf( "(controller.cpp) ERROR: main() failed calling log_c.open() on FD_ERROR_LOG\nExiting\n" );
        exit( 1 );
    }

    if( open_time_log( ) != ERROR_NONE ) {
        error_log.close( );
        printf( "(controller.cpp) init() failed calling open_time_log()\nExiting\n" );
        exit( 1 );
    }


    // Get the cpu speed
    if( read_cpuinfo( ) != ERROR_NONE ) {
        error_log.close( );
        printf( "(controller.cpp) init() failed calling read_cpuinfo()\nExiting\n" );
        exit( 1 );
    }

    // connect to the shared message buffer BEFORE attempting any IPC
    // Note: the message buffer is created by the coordinator before the controller
    // is launched
    amsgbuffer = actuator_msg_buffer_c( ACTUATOR_MSG_BUFFER_NAME, ACTUATOR_MSG_BUFFER_MUTEX_NAME );
    if( amsgbuffer.open( ) != BUFFER_ERROR_NONE ) {
        sprintf( strbuffer, "(controller.cpp) init(argc,argv) failed calling actuator_msg_buffer_c.open(...,false)\n" );
        error_log.write( strbuffer );
        error_log.close( );
        exit( 1 );
    }

}

//-----------------------------------------------------------------------------

/// Standalone Controller Entry Point
// TODO : refactor to exit as gracefully as possible
int main( int argc, char* argv[] ) {

    actuator_msg_c state, command;
    int cycle = 0;
    controller_notification_c notification;
    unsigned long long ts;
    const Real INTERVAL = 1.0 / (double) CONTROLLER_FREQUENCY_HERTZ;
    Real t = 0.0;

    // Note: if init fails, the controller bombs out.  Will write a message to
    // console and/or error_log if this happens.
    init( );

    // query and publish initial values
    get_initial_state( );

    notification.type = CONTROLLER_NOTIFICATION_SNOOZE;
    notification.duration = INTERVAL;

    // start the main process loop
    while( 1 ) {

        // get the current timestamp
        rdtscll( ts );
        notification.ts = ts;

        printf( "(controller) writing notification\n" );
        // send a snooze notification
        write( FD_CONTROLLER_TO_COORDINATOR_WRITE_CHANNEL, &notification, sizeof( controller_notification_c ) );

        // unsnoozed here, so get a new timestamp
        rdtscll( ts );
        // Note: may have some long term drift due to dp math
        t += INTERVAL;

        printf( "(controller) getting state\n" );

        // get the state (via dynamics)
        if( get_state( t, state ) != ERROR_NONE ) {
            sprintf( strbuffer, "(controller.cpp) main() failed calling get_state(%10.9f,state)\n", t );
            error_log.write( strbuffer );
            //TODO : determine appropriate response
        }

        // build a command
        if( control( state, command ) != ERROR_NONE ) {
            sprintf( strbuffer, "(controller.cpp) main() failed calling control(state,command) at time %10.9f\n", t );
            error_log.write( strbuffer );
            //TODO : determine appropriate response
        }
        command.header.type = ACTUATOR_MSG_COMMAND;

        // publish the command
        publish_command( command );
        if( publish_command( command ) != ERROR_NONE ) {
            sprintf( strbuffer, "(controller.cpp) main() failed calling publish_command(command) at time %10.9f\n", t );
            error_log.write( strbuffer );
            //TODO : determine appropriate response
        }

        cycle++;
    }

    amsgbuffer.close( );  // TODO : figure out a way to force a clean up

    return 0;
}
/*
int main( int argc, char* argv[] ) {

    actuator_msg_c state, command;
    int cycle = 0;

    unsigned long long cy_interval, cy_start, cy_control, cy_interval_end, cy_interval_start, cy_current;

    // Note: if init fails, the controller bombs out.  Will write a message to
    // console and/or error_log if this happens.
    init( );

    // compute the controller's interval
    assert( CONTROLLER_FREQUENCY_HERTZ > 1 && CONTROLLER_FREQUENCY_HERTZ < USECS_PER_SEC );
    cy_interval = cpu_speed_hz / CONTROLLER_FREQUENCY_HERTZ;

    // query and publish initial values
    get_initial_state( );

    // now that initialization is complete record start time (for interval computation)
    rdtscll( cy_start );

    // record the start time as the start of the initial interval
    cy_interval_start = cy_start;

    // compute the end of the initial interval
    cy_interval_end = cy_interval_start + cy_interval;

    // start the main process loop
    while( 1 ) {
        // poll time
        // Note: double edged: poll may block on system call, but using signal
        // will definitely block and adds complexity that may invalidate the
        // coordinator measuring of the controller
        rdtscll( cy_current );

        // compare the current time with the end time
    if( cy_current < cy_interval_end ) continue;

        // otherwise, the interval has been exceeded

        // compute the actual time to compute the control output
        // Note: current is absolute from beginning of process
        // but control needs to be computed in terms of the time
        // after the controller was initialized
        cy_control = cy_current - cy_start;

        // Compute the time
        Real t = cycles_to_seconds( cy_control, cpu_speed_hz );
        //if( VERBOSE ) printf( "(controller) Requesting state at time: %10.9f\n", t );
        //printf( "(controller) Requesting state at time: %10.9f\n", t );

        // get the state (via dynamics)
        if( get_state( t, state ) != ERROR_NONE ) {
            sprintf( strbuffer, "(controller.cpp) main() failed calling get_state(%10.9f,state)\n", t );
            error_log.write( strbuffer );
            //TODO : determine appropriate response
        }

        // build a command
        if( control( state, command ) != ERROR_NONE ) {
            sprintf( strbuffer, "(controller.cpp) main() failed calling control(state,command) at time %10.9f\n", t );
            error_log.write( strbuffer );
            //TODO : determine appropriate response
        }
        command.header.type = ACTUATOR_MSG_COMMAND;

        // publish the command
        publish_command( command );
        if( publish_command( command ) != ERROR_NONE ) {
            sprintf( strbuffer, "(controller.cpp) main() failed calling publish_command(command) at time %10.9f\n", t );
            error_log.write( strbuffer );
            //TODO : determine appropriate response
        }

        // gather stats on actual duration of the last interval
//        timersub( &tv_current, &tv_interval_start, &tv_interval_elapsed );
//        timersub( &tv_current, &tv_interval_end, &tv_interval_error );

//        if( VERBOSE ) printf( "(controller) Actual Interval: %e\n", timeval_to_real( tv_interval_elapsed ) );
//        if( VERBOSE ) printf( "(controller) Interval Error: %e\n", timeval_to_real( tv_interval_error ) );

        // recompute start of the interval
        // Note: the interval is computed on the time it was supposed to end
        // not on the current time.  It is highly unlikely that the current will
        // be on time (at the end) but the error of assuming the actual time
        // accumulates.  So using the actual value instead of expected.
    cy_interval_start = cy_current;

        // recompute the end of the interval
    cy_interval_end = cy_interval_start + cy_interval;

//        if( VERBOSE ) printf( "(controller) Next interval length: %f\n", timeval_to_real( tv_interval ) );
//        if( VERBOSE ) printf( "(controller) Next interval start: %f\n", timeval_to_real( tv_interval_start ) );
//        if( VERBOSE ) printf( "(controller) Next interval end: %f\n", timeval_to_real( tv_interval_end ) );
//        if( VERBOSE ) printf( "(controller) Current time: %f\n", timeval_to_real( tv_current ) );

        // TEST: to be commented.  Validate the wakeup event in coordinator.
        // usleep(100);			// Sanity check

        cycle++;
    }

    amsgbuffer.close( );  // TODO : figure out a way to force a clean up

    return 0;
}
*/

/*
int main( int argc, char* argv[] ) {

    actuator_msg_c state, command;
    int cycle = 0;
    struct timeval tv_start, tv_current, tv_control;
    struct timeval tv_interval, tv_interval_start, tv_interval_end;
    struct timeval tv_interval_elapsed, tv_interval_error;

    // Note: if init fails, the controller bombs out.  Will write a message to
    // console and/or error_log if this happens.
    init( );

    // compute the controller's interval
    tv_interval.tv_sec = 0;
    assert( CONTROLLER_FREQUENCY_HERTZ > 1 && CONTROLLER_FREQUENCY_HERTZ < USECS_PER_SEC );
    tv_interval.tv_usec = USECS_PER_SEC / CONTROLLER_FREQUENCY_HERTZ;

    // query and publish initial values
    get_initial_state( );

    // now that initialization is complete record start time (for interval computation)
    tv_start = get_process_time( );

    // record the start time as the start of the initial interval
    memcpy( &tv_interval_start, &tv_start, sizeof(timeval) );

    // compute the end of the initial interval
    timeradd( &tv_interval_start, &tv_interval, &tv_interval_end );

    // start the main process loop
    while( 1 ) {
        // poll time
        // Note: double edged: poll may block on system call, but using signal
        // will definitely block and adds complexity that may invalidate the
        // coordinator measuring of the controller
        tv_current = get_process_time( );

        // compare the current time with the end time
        int result = timercmp( &tv_current, &tv_interval_end, >= );

        // if the comparison is true (tv_current is neither greater than nor EQUAL TO) then poll again
        if( !result )	continue;

        // otherwise, the interval has been exceeded

        // **Question** For the time at which the control is issued,
        // should we use the time that actually elapsed or should we
        // use the time that the controller was supposed to fire.  This
        // is polling after all and these times are very likely to be
        // different.  I have arbitrarily chosen the actual time as
        // opposed to the expected time.

        // compute the actual time to compute the control output
        // Note: current is absolute from beginning of process
        // but control needs to be computed in terms of the time
        // after the controller was initialized
        timersub( &tv_current, &tv_start, &tv_control );

        // Compute the time
        Real t = timeval_to_real( tv_control );
        //if( VERBOSE ) printf( "(controller) Requesting state at time: %10.9f\n", t );
        //printf( "(controller) Requesting state at time: %10.9f\n", t );

        // get the state (via dynamics)
        if( get_state( t, state ) != ERROR_NONE ) {
            sprintf( strbuffer, "(controller.cpp) main() failed calling get_state(%10.9f,state)\n", t );
            error_log.write( strbuffer );
            //TODO : determine appropriate response
        }

        // build a command
        if( control( state, command ) != ERROR_NONE ) {
            sprintf( strbuffer, "(controller.cpp) main() failed calling control(state,command) at time %10.9f\n", t );
            error_log.write( strbuffer );
            //TODO : determine appropriate response
        }
        command.header.type = ACTUATOR_MSG_COMMAND;

        // publish the command
        publish_command( command );
        if( publish_command( command ) != ERROR_NONE ) {
            sprintf( strbuffer, "(controller.cpp) main() failed calling publish_command(command) at time %10.9f\n", t );
            error_log.write( strbuffer );
            //TODO : determine appropriate response
        }

        // gather stats on actual duration of the last interval
        timersub( &tv_current, &tv_interval_start, &tv_interval_elapsed );
        timersub( &tv_current, &tv_interval_end, &tv_interval_error );

        if( VERBOSE ) printf( "(controller) Actual Interval: %e\n", timeval_to_real( tv_interval_elapsed ) );
        if( VERBOSE ) printf( "(controller) Interval Error: %e\n", timeval_to_real( tv_interval_error ) );

        // recompute start of the interval
        // Note: the interval is computed on the time it was supposed to end
        // not on the current time.  It is highly unlikely that the current will
        // be on time (at the end) but the error of assuming the actual time
        // accumulates.  So using the actual value instead of expected.
        memcpy( &tv_interval_start, &tv_current, sizeof(timeval) );

        // recompute the end of the interval
        timeradd( &tv_interval_start, &tv_interval, &tv_interval_end );

        if( VERBOSE ) printf( "(controller) Next interval length: %f\n", timeval_to_real( tv_interval ) );
        if( VERBOSE ) printf( "(controller) Next interval start: %f\n", timeval_to_real( tv_interval_start ) );
        if( VERBOSE ) printf( "(controller) Next interval end: %f\n", timeval_to_real( tv_interval_end ) );
        if( VERBOSE ) printf( "(controller) Current time: %f\n", timeval_to_real( tv_current ) );

        // TEST: to be commented.  Validate the wakeup event in coordinator.
        // usleep(100);			// Sanity check

        cycle++;
    }

    amsgbuffer.close( );  // TODO : figure out a way to force a clean up

    return 0;
}
*/
//-----------------------------------------------------------------------------
