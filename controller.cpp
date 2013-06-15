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

    //Real desired_position = position_trajectory( time, 0.5, 0.0, PI );
    //Real desired_velocity = velocity_trajectory( time, 0.5, 0.0, PI );

    Real desired_position = position_trajectory( time, 10.0, 0.0, PI );
    Real desired_velocity = velocity_trajectory( time, 10.0, 0.0, PI );

    reply.command.torque = Kp * ( desired_position - measured_position ) + Kv * ( desired_velocity - measured_velocity );

    return reply;
}

//-----------------------------------------------------------------------------

ActuatorMessage get_state( const Real& time ) {

    char buf = 0;
    ActuatorMessage request;
    ActuatorMessage state;
        
    request.header.type = MSG_TYPE_STATE_REQUEST;
    request.state.time = time;

    amsgbuffer.write( request );

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

    // At this point a message was sent from the coordinator.  Attempt to read from buffer
    // read the state out of the buffer
    // Note: will block waiting to acquire mutex
    state = amsgbuffer.read( );

    return state;
}

//-----------------------------------------------------------------------------

void publish_command( ActuatorMessage cmd ) {

    char buf = 0;

    // send the command message to the actuator
    amsgbuffer.write( cmd );
    // Note: will block waiting to acquire mutex
	
    // send a notification to the coordinator
    write( FD_CONTROLLER_TO_COORDINATOR_WRITE_CHANNEL, &buf, 1 );

}

//-----------------------------------------------------------------------------

void request_initial_state_then_publish_initial_command( void ) {

    ActuatorMessage state = get_state( 0.0 );

    if( VERBOSE ) printf( "(controller) received initial state\n" );
    if( VERBOSE ) state.print();

    // compute the command message
    ActuatorMessage command = control( state );
    command.header.type = MSG_TYPE_COMMAND;

    if( VERBOSE ) printf( "(controller) publishing initial command\n" );
    publish_command( command );
}

//-----------------------------------------------------------------------------

int fd_timer_to_controller[2];

void initialize_pipes( void ) {
    int flags;

    // NOTE: a read channel needs to be blocking to force  block waiting for event
    // NOTE: a write channel should be non-blocking (won't ever fill up buffer anyway)

    if( pipe( fd_timer_to_controller ) != 0 ) {
        throw std::runtime_error( "Failed to open fd_timer_to_controller pipe." ) ;
    }
    flags = fcntl( fd_timer_to_controller[0], F_GETFL, 0 );
    fcntl( fd_timer_to_controller[0], F_SETFL, flags );
    flags = fcntl( fd_timer_to_controller[1], F_GETFL, 0 );
    fcntl( fd_timer_to_controller[1], F_SETFL, flags | O_NONBLOCK );
}

//-----------------------------------------------------------------------------

struct timeval get_process_time( void ) {
    struct rusage ru;
    struct timeval tv;

    //int result = getrusage( RUSAGE_SELF, &ru );
    //memcpy( &tv, &ru.ru_utime, sizeof(timeval) );

    gettimeofday( &tv, NULL );

    return tv;
}

//-----------------------------------------------------------------------------

Real timeval_to_real( const struct timeval& tv ) {
    return (Real) tv.tv_sec + (Real) tv.tv_usec / (Real) USECS_PER_SEC;
}

//-----------------------------------------------------------------------------


// The signal identifier for the controller's real-time timer
#define RTTIMER_SIGNAL                          SIGRTMIN + 5

// timer variables
clockid_t _clock;
sigset_t rttimer_mask;

// The signal action to reference controller's process signal handler
struct sigaction rttimer_sigaction;

#define ERR_TIMER_NOERROR                       0
#define ERR_TIMER_FAILURE_SIGACTION             1
#define ERR_TIMER_FAILURE_SIGPROCMASK           2
#define ERR_TIMER_FAILURE_CREATE                3
#define ERR_TIMER_FAILURE_SETTIME               4
#define ERR_TIMER_FAILURE_GETCLOCKID            5

void interval_sighandler( int signum, siginfo_t *si, void *data ) {
    printf( "(controller) timer event\n" );
    char buf = 0;
    write( fd_timer_to_controller[1], &buf, 1 );
}

int block_signal_rttimer( void ) {
    if( sigprocmask( SIG_SETMASK, &rttimer_mask, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGPROCMASK;
    return ERR_TIMER_NOERROR;
}

int unblock_signal_rttimer( void ) {
    if( sigprocmask( SIG_UNBLOCK, &rttimer_mask, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGPROCMASK;
    return ERR_TIMER_NOERROR;
}

int create_rttimer_monitor( void ) {
    timer_t timerid;
    struct sigevent sevt;
    struct itimerspec its;

    // Establish handler for timer signal
    rttimer_sigaction.sa_flags = SA_SIGINFO;
    rttimer_sigaction.sa_sigaction = interval_sighandler;
    sigemptyset( &rttimer_sigaction.sa_mask );
    if( sigaction( RTTIMER_SIGNAL, &rttimer_sigaction, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGACTION;

    // intialize the signal mask
    sigemptyset( &rttimer_mask );
    sigaddset( &rttimer_mask, RTTIMER_SIGNAL );

    // Block timer signal temporarily
    if( block_signal_rttimer() != ERR_TIMER_NOERROR )
        return ERR_TIMER_FAILURE_SIGPROCMASK;

    if( clock_getcpuclockid( getpid( ), &_clock ) != 0 )
        return ERR_TIMER_FAILURE_GETCLOCKID;

    // Create the timer
    sevt.sigev_notify = SIGEV_SIGNAL;
    sevt.sigev_signo = RTTIMER_SIGNAL;
    sevt.sigev_value.sival_ptr = &timerid;
    if( timer_create( _clock, &sevt, &timerid ) == -1 )
       return ERR_TIMER_FAILURE_CREATE;

    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = 1;
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 1000000;

    // Set up the timer
    if( timer_settime( timerid, 0, &its, NULL ) == -1 )
        return ERR_TIMER_FAILURE_SETTIME;

    // NOTE: The timer is blocked at this point.  It must be unblocked to function.

    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------

/// Standalone Controller Entry Point
int main( int argc, char* argv[] ) {

    ActuatorMessage state, command;
    int cycle = 0;
    // Real time = 0.0;
    struct timeval tv_start, tv_current, tv_control;
    struct timeval tv_interval, tv_interval_start, tv_interval_end, tv_interval_elapsed;

    unsigned long long rdtsc_mark1;
    unsigned long long rdtsc_mark2;

    tv_interval.tv_sec = 0;
    tv_interval.tv_usec = 1000;

    //tv_interval.tv_sec = 1;
    //tv_interval.tv_usec = 0;

    // connect to the shared message buffer BEFORE attempting any IPC
    amsgbuffer = ActuatorMessageBuffer( ACTUATOR_MESSAGE_BUFFER_NAME, ACTUATOR_MESSAGE_BUFFER_MUTEX_NAME );
    amsgbuffer.open( );

    // ----------
    initialize_pipes( );
    create_rttimer_monitor( );
    // ----------

    // query and publish initial values
    request_initial_state_then_publish_initial_command( );

    // now that initialization is complete record start time (for interval computation)
    tv_start = get_process_time( );

    // record the start time as the start of the initial interval
    memcpy( &tv_interval_start, &tv_start, sizeof(timeval) );

    // compute the end of the initial interval
    timeradd( &tv_interval_start, &tv_interval, &tv_interval_end );

    // ----------
    char buf = 0;
    unblock_signal_rttimer( );
    // ----------

    // start the main process loop
    while( 1 ) {
        if( read( fd_timer_to_controller[0], &buf, 1 ) == -1 ) {
            perror( "read()" );
        }

        // poll time
        // Note: double edged: poll may block on system call, but using signal
        // will definitely block and adds complexity that may invalidate the
        // coordinator measuring of the controller
        tv_current = get_process_time( );

        // compute the actual time to compute the control output
        // Note: current is absolute from beginning of process
        // but control needs to be computed in terms of the time
        // after the controller was initialized
        timersub( &tv_current, &tv_start, &tv_control );

        // Compute the time
        Real t = timeval_to_real( tv_control );
        if( VERBOSE ) printf( "(controller) Requesting state at time: %f\n", t );

        state = get_state( t );

        command = control( state );
        command.header.type = MSG_TYPE_COMMAND;

        if( VERBOSE ) printf( "(controller) Publishing command for time: %f\n", t );
        publish_command( command );

        // recompute start of the interval
        // Note: the interval is computed on the time it was supposed to end
        // not on the current time.  It is highly unlikely that the current will
        // be on time (at the end) and the error could accumulate as a result
        // so using the expected value instead of actual.  Error in micorseconds
        memcpy( &tv_interval_start, &tv_interval_end, sizeof(timeval) );

        // recompute the end of the interval
        timeradd( &tv_interval_start, &tv_interval, &tv_interval_end );

        if( VERBOSE ) printf( "(controller) Current: %f\n", timeval_to_real( tv_current ) );
        if( VERBOSE ) printf( "(controller) Interval: %f\n", timeval_to_real( tv_interval ) );
        if( VERBOSE ) printf( "(controller) Next interval start: %f\n", timeval_to_real( tv_interval_start ) );
        if( VERBOSE ) printf( "(controller) Next interval end: %f\n", timeval_to_real( tv_interval_end ) );

        cycle++;

/*
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
	
        // compute the actual time to compute the control output
        // Note: current is absolute from beginning of process
        // but control needs to be computed in terms of the time
        // after the controller was initialized
    	timersub( &tv_current, &tv_start, &tv_control );

        // Compute the time
        Real t = timeval_to_real( tv_control );
        if( VERBOSE ) printf( "(controller) Requesting state at time: %f\n", t );

        state = get_state( t );
    
        command = control( state );
        command.header.type = MSG_TYPE_COMMAND;

        if( VERBOSE ) printf( "(controller) Publishing command for time: %f\n", t );
        publish_command( command );

        // recompute start of the interval
        // Note: the interval is computed on the time it was supposed to end
        // not on the current time.  It is highly unlikely that the current will
        // be on time (at the end) and the error could accumulate as a result
        // so using the expected value instead of actual.  Error in micorseconds
        memcpy( &tv_interval_start, &tv_interval_end, sizeof(timeval) );

        // recompute the end of the interval
    	timeradd( &tv_interval_start, &tv_interval, &tv_interval_end );

        if( VERBOSE ) printf( "(controller) Current: %f\n", timeval_to_real( tv_current ) );
        if( VERBOSE ) printf( "(controller) Interval: %f\n", timeval_to_real( tv_interval ) );
        if( VERBOSE ) printf( "(controller) Next interval start: %f\n", timeval_to_real( tv_interval_start ) );
        if( VERBOSE ) printf( "(controller) Next interval end: %f\n", timeval_to_real( tv_interval_end ) );

        cycle++;
*/
    }

    amsgbuffer.close( );  // TODO : figure out a way to force a clean up

    return 0;
}

//-----------------------------------------------------------------------------
