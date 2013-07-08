/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

coordinator.cpp
-----------------------------------------------------------------------------*/

#include <tas/tas.h>
#include <tas/time.h>
#include <tas/dynamics_plugin.h>
#include <tas/cpu.h>
#include <tas/schedule.h>
#include <tas/actuator.h>
#include <tas/log.h>
#include <tas/experiment.h>

//-----------------------------------------------------------------------------

using boost::dynamic_pointer_cast;

//-----------------------------------------------------------------------------

timestamp_buffer_c timestamp_timer_buffer;

//-----------------------------------------------------------------------------

// timer variables
sigset_t rttimer_mask;

// The signal action to reference controller's process signal handler
struct sigaction rttimer_sigaction;
timer_t rttimer_id;

//-----------------------------------------------------------------------------

// Coordinator process information
static pid_t coordinator_pid;
static int coordinator_priority;

// Controller process information
static int controller_pid;

bool controller_is_blocked;
bool controller_is_suspended;

bool controller_fully_initialized = false;

unsigned long long CONTROLLER_PERIOD_NSEC;

//-----------------------------------------------------------------------------
// CPU
//-----------------------------------------------------------------------------

#define DEFAULT_PROCESSOR    0

//-----------------------------------------------------------------------------

//cpuinfo_c cpuinfo;
//double cpu_speed_mhz;
unsigned long long cpu_speed_hz;

//-----------------------------------------------------------------------------
// LOGGING
//-----------------------------------------------------------------------------

log_c variance_log;
log_c error_log;
char errstr[ 256 ];

//-----------------------------------------------------------------------------

error_e open_error_log( void ) {
    error_log = log_c( "error.log" );
    //error_log = log_c( LOG_CHANNEL_STDERR );

    if( error_log.open( ) != LOG_ERROR_NONE ) {
        printf( "(coordinator.cpp) ERROR: open_error_log() failed calling log_c.open() on file error.log\n" );
        return ERROR_FAILED;
    }

    // dup the error log fd into a known channel so shared resources
    // know exactly what fd the log file is using
    if( dup2( error_log.get_fd(), FD_ERROR_LOG ) == -1 ) {
        printf( "(coordinator.cpp) ERROR: open_error_log() failed calling dup2(error_log.get_fd(), FD_ERROR_LOG)\n" );
	return ERROR_FAILED;
    }

    return ERROR_NONE;
}

//-----------------------------------------------------------------------------

error_e open_variance_log( void ) {
    variance_log = log_c( "variance.dat" );
    //variance_log = log_c( LOG_CHANNEL_STDOUT );

    if( variance_log.open( ) != LOG_ERROR_NONE ) {
        sprintf( errstr, "(coordinator.cpp) open_variance_log() failed calling log_c.open() on file variance.dat\n" );
        error_log.write( errstr );
        return ERROR_FAILED;
    }
    return ERROR_NONE;
}

//-----------------------------------------------------------------------------
// SHARED RESOURCE MANAGEMENT
//-----------------------------------------------------------------------------

// The shared message buffer
actuator_msg_buffer_c amsgbuffer;

//-----------------------------------------------------------------------------
// PLUGIN MANAGEMENT
//-----------------------------------------------------------------------------

// list of plugins.
// Note: In actuality only support a single plugin at this time.
std::list< dynamics_plugin_c > plugins;

//-----------------------------------------------------------------------------

/**
  invokes the intialization function (init_f) of all registered dynamics plugins by
  forwarding command line parameters to the function
*/
/*
void coordinator_init_dynamics( int argc, char** argv ) {
    for( std::list< DynamicsPlugin >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it->init)( argc, argv );
    }
}
*/
//-----------------------------------------------------------------------------

/**
  invokes the step function (step_f) of all the registered dynamics plugins
*/
void step_dynamics( Real dt ) {

    for( std::list< dynamics_plugin_c >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it->step)( dt );
    }
}

//-----------------------------------------------------------------------------

void shutdown_dynamics( void ) {

    for( std::list< dynamics_plugin_c >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it->shutdown)( );
    }
}

//-----------------------------------------------------------------------------

/**
  a means to publish the state from dynamics is necessary (at least in this architecture)
  because there is a need to seed data to the controller (prime the controller)
  so everything can start.  Once everything is started though, publish state is
  really wrapped into the run_dynamics mechanism
*/
void dynamics_state_requested( void ) {

    for( std::list< dynamics_plugin_c >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it->write_state)( );
    }
}

//-----------------------------------------------------------------------------

void dynamics_get_command( void ) {

    for( std::list< dynamics_plugin_c >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it->read_command)( );
    }
}

//-----------------------------------------------------------------------------
// TODO : Improve error handling
error_e init_dynamics( const int& argc, char* argv[] ) {

    dynamics_plugin_c plugin;
    int result = plugin.read( DYNAMICS_PLUGIN );	// TODO: Sanity/Safety checking
    plugin.init( argc, argv );

    // if both functions were located then push the pair into the list of plugins
    if( !result ) plugins.push_back( plugin );

    return ERROR_NONE;
}


//-----------------------------------------------------------------------------
// INTERPROCESS COMMUNICATION (IPC)
//-----------------------------------------------------------------------------

int fd_timer_to_coordinator[2];		// channel from timer to coordinator. Timer event
int fd_wakeup_to_coordinator[2];	// channel from wakeup to coordinator. Wakeup event
int fd_coordinator_to_controller[2];	// channel from coordinator to controller. Notification
int fd_controller_to_coordinator[2];	// channel from controller to coordinator. Notification

//-----------------------------------------------------------------------------

/// Initializes all interprocess communication channels.  This must be done
/// before any controller processes are forked or threads are spawned so that they
/// will inherit the file descriptors from the coordinator.
error_e init_pipes( void ) {
    int flags;

    // NOTE: a read channel needs to be blocking to force  block waiting for event
    // NOTE: a write channel should be non-blocking (won't ever fill up buffer anyway)

    // Open the timer to coordinator channel for timer events
    if( pipe( fd_timer_to_coordinator ) != 0 ) {
        sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling pipe(fd_timer_to_coordinator)\n" );
        error_log.write( errstr );
        return ERROR_FAILED;
    }
    flags = fcntl( fd_timer_to_coordinator[0], F_GETFL, 0 );
    fcntl( fd_timer_to_coordinator[0], F_SETFL, flags );
    flags = fcntl( fd_timer_to_coordinator[1], F_GETFL, 0 );
    fcntl( fd_timer_to_coordinator[1], F_SETFL, flags | O_NONBLOCK );

    // Open the wakeup to coordinator channel for wakeup (blocking) events
    if( pipe( fd_wakeup_to_coordinator ) != 0 ) {
        close( fd_timer_to_coordinator[0] );
        close( fd_timer_to_coordinator[1] );
        sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling pipe(fd_wakeup_to_coordinator)\n" );
        error_log.write( errstr );
        return ERROR_FAILED;
    }
    flags = fcntl( fd_wakeup_to_coordinator[0], F_GETFL, 0 );
    fcntl( fd_wakeup_to_coordinator[0], F_SETFL, flags );
    flags = fcntl( fd_wakeup_to_coordinator[1], F_GETFL, 0 );
    //fcntl( fd_wakeup_to_coordinator[1], F_SETFL, flags | O_NONBLOCK );
    fcntl( fd_wakeup_to_coordinator[1], F_SETFL, flags );

    // Open the coordinator to controller channel for notifications
    if( pipe( fd_coordinator_to_controller ) != 0 ) {
        close( fd_timer_to_coordinator[0] );
        close( fd_timer_to_coordinator[1] );
        close( fd_wakeup_to_coordinator[0] );
        close( fd_wakeup_to_coordinator[1] );
        sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling pipe(fd_coordinator_to_controller)\n" );
        error_log.write( errstr );
        return ERROR_FAILED;
    }
    flags = fcntl( fd_coordinator_to_controller[0], F_GETFL, 0 );
    fcntl( fd_coordinator_to_controller[0], F_SETFL, flags );
    flags = fcntl( fd_coordinator_to_controller[1], F_GETFL, 0 );
    fcntl( fd_coordinator_to_controller[1], F_SETFL, flags | O_NONBLOCK );

    // Open the controller to coordinator channel for notifications
    if( pipe( fd_controller_to_coordinator ) != 0 ) {
        close( fd_timer_to_coordinator[0] );
        close( fd_timer_to_coordinator[1] );
        close( fd_wakeup_to_coordinator[0] );
        close( fd_wakeup_to_coordinator[1] );
        close( fd_coordinator_to_controller[0] );
        close( fd_coordinator_to_controller[1] );
        sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling pipe(fd_controller_to_coordinator)\n" );
        error_log.write( errstr );
        return ERROR_FAILED;
    }
    flags = fcntl( fd_controller_to_coordinator[0], F_GETFL, 0 );
    fcntl( fd_controller_to_coordinator[0], F_SETFL, flags );
    flags = fcntl( fd_controller_to_coordinator[1], F_GETFL, 0 );
    fcntl( fd_controller_to_coordinator[1], F_SETFL, flags | O_NONBLOCK );

    return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Clean up the pipes to prevent leakage
void close_pipes( void ) {
    close( fd_timer_to_coordinator[0] );
    close( fd_timer_to_coordinator[1] );
    close( fd_wakeup_to_coordinator[0] );
    close( fd_wakeup_to_coordinator[1] );
    close( fd_coordinator_to_controller[0] );
    close( fd_coordinator_to_controller[1] );
    close( fd_controller_to_coordinator[0] );
    close( fd_controller_to_coordinator[1] );
}

//-----------------------------------------------------------------------------
// PROCESS MANAGEMENT
//-----------------------------------------------------------------------------


/// Creates the controller process with a priority level one step below the coordinator.
/// Note: an ERROR_NONE in the coordinator does not really guarantee that the controller
/// started properly
error_e fork_controller( void ) {

    controller_pid = fork( );

    // handle any fork error
    if( controller_pid < 0 ) {
        sprintf( errstr, "(coordinator.cpp) fork_controller() failed calling fork()\n" );
        error_log.write( errstr );
        return ERROR_FAILED;
    }

    // start of the contoller process
    if( controller_pid == 0 ) {

        controller_pid = getpid( );
        int controller_priority;

        set_cpu( controller_pid, DEFAULT_PROCESSOR );
        set_realtime_schedule_rel( controller_pid, controller_priority, 1 );

        if( set_cpu( controller_pid, DEFAULT_PROCESSOR ) != ERROR_NONE ) {
            sprintf( errstr, "(coordinator.cpp) fork_controller() failed calling set_cpu(coordinator_pid,DEFAULT_PROCESSOR).\nExiting\n" );
            error_log.write( errstr );
            error_log.close( );
            printf( "%s", errstr );
            return ERROR_FAILED;
        }

        if( set_realtime_schedule_rel( controller_pid, controller_priority, 1 ) != ERROR_NONE ) {
            sprintf( errstr, "(coordinator.cpp) fork_controller() failed calling set_realtime_schedule_rel(controller_pid,controller_priority,1).\nExiting\n" );
            error_log.write( errstr );
            error_log.close( );
            printf( "%s", errstr );
            return ERROR_FAILED;
        }

        printf( "controller process priority: %d\n", controller_priority );

        //+++++++++++++++++++++++++++++++++
        // set up explicit IPC interface
        
        // duplicate the default fd's into known fd's so that the controller explicitly knows
        // what the channels are.
        // Note: dup2 explicitly closes the channel you are duping into so there is no need
        // for a seperate close call preceding the dup2 call unless some kernel programmer
        // didn't meet the spec.
        // TODO : error handling
        dup2( fd_coordinator_to_controller[0], FD_COORDINATOR_TO_CONTROLLER_READ_CHANNEL );
        dup2( fd_controller_to_coordinator[1], FD_CONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );

        // close the ends of the pipes that the controller is not allowed to use
        close( fd_coordinator_to_controller[1] );   // close the write end for the controller
        close( fd_controller_to_coordinator[0] );   // close the read end for the controller

        // close the other pipes that the controller should have no knowledge of
        close( fd_timer_to_coordinator[0] );
        close( fd_timer_to_coordinator[1] );
        close( fd_wakeup_to_coordinator[0] );
        close( fd_wakeup_to_coordinator[1] );

        // execute the controller program
        // TODO : error handling
        execl( CONTROLLER_PROGRAM, "controller", NULL );

        // exit on fail to exec.  Can only get here if execl fails.
        _exit( 1 );
    }

    return ERROR_NONE;
}

//-----------------------------------------------------------------------------

/// Pause the controller from the coordinator process
void suspend_controller( void ) {

    kill( controller_pid, SIGSTOP );

    controller_is_suspended = true;
}

//-----------------------------------------------------------------------------

/// Continue the controller from the coordinator process
void resume_controller( void ) {

    controller_is_suspended = false;

    kill( controller_pid, SIGCONT );
}

//-----------------------------------------------------------------------------

void shutdown_controller( void ) {

    kill( controller_pid, SIGTERM );
    //kill( controller_pid, SIGKILL );
}

//-----------------------------------------------------------------------------
// TIMER CONTROL
//-----------------------------------------------------------------------------

/// The signal handler for the controller's timer.  Is called only when the controller's
/// budget is exhausted.  Puts a message out onto the comm channel the coordinator
/// is listening to for notifications about this event
void rttimer_sighandler( int signum, siginfo_t *si, void *data ) {

    unsigned long long ts;

    rdtscll( ts );

    if( write( fd_timer_to_coordinator[1], &ts, sizeof(unsigned long long) ) == -1) {
        std::string err_msg = "(coordinator.cpp) rttimer_sighandler(...) failed making system call write(...)";
        error_log.write( error_string_bad_write( err_msg, errno ) );
        // TODO : determine if there is a need to recover
    }

    // get overrun!?!
}

//-----------------------------------------------------------------------------

/// Blocks the timer signal if it is necessary to suppress the timer
timer_err_e block_rttimer( void ) {
    if( sigprocmask( SIG_SETMASK, &rttimer_mask, NULL ) == -1 )
        return TIMER_ERROR_SIGPROCMASK;
    return TIMER_ERROR_NONE;
}

//-----------------------------------------------------------------------------

/// Unblocks a blocked timer signal
timer_err_e unblock_rttimer( void ) {
    if( sigprocmask( SIG_UNBLOCK, &rttimer_mask, NULL ) == -1 )
        return TIMER_ERROR_SIGPROCMASK;
    return TIMER_ERROR_NONE;
}

//-----------------------------------------------------------------------------

timer_err_e arm_timer( const unsigned long long& ts_req, unsigned long long& ts_arm ) {

    struct itimerspec its;
    unsigned long long ts_now;

    // sanity check.  Not supporting controllers that run at 1 Hz or slower or faster than 1ns.
    assert( CONTROLLER_HZ > 1 && CONTROLLER_HZ <= NSECS_PER_SEC );

    rdtscll( ts_now );

    unsigned long long dts = ts_now - ts_req;
    unsigned long long dns = cycles_to_nanoseconds( dts, cpu_speed_hz );
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 0;
    its.it_value.tv_sec = 0;
    //its.it_value.tv_nsec = NSECS_PER_SEC / CONTROLLER_HZ;
    its.it_value.tv_nsec = (unsigned long long)(CONTROLLER_PERIOD_NSEC - dns);

    //if( timer_settime( rttimer_id, TIMER_ABSTIME, &its, NULL ) == -1 )
    if( timer_settime( rttimer_id, 0, &its, NULL ) == -1 ) {
	printf( "dts: %lld, dns: %lld, nsec: %d\n", dts, dns, its.it_value.tv_nsec );
        return TIMER_ERROR_SETTIME;
    }

    //printf( "%lld, %lld\n", dts, dns );

    // timestamp
    //rdtscll( ts_arm );
    ts_arm = ts_req;

    return TIMER_ERROR_NONE;
}

//-----------------------------------------------------------------------------

/// Creates a high-resolution real-time timer to monitor the controller process
timer_err_e create_hrrttimer( void ) {
    struct sigevent sevt;
    struct itimerspec its;

    // Establish handler for timer signal
    rttimer_sigaction.sa_flags = SA_SIGINFO; //<-
    rttimer_sigaction.sa_sigaction = rttimer_sighandler;
    sigemptyset( &rttimer_sigaction.sa_mask );  //<-
    if( sigaction( RTTIMER_SIGNAL, &rttimer_sigaction, NULL ) == -1)
        return TIMER_ERROR_SIGACTION;

    // intialize the signal mask
    sigemptyset( &rttimer_mask );
    sigaddset( &rttimer_mask, RTTIMER_SIGNAL );

    sevt.sigev_notify = SIGEV_SIGNAL;
    sevt.sigev_signo = RTTIMER_SIGNAL;
    sevt.sigev_value.sival_ptr = &rttimer_id;
    if( timer_create( CLOCK_MONOTONIC, &sevt, &rttimer_id ) == -1 )
    //if( timer_create( CLOCK_REALTIME, &sevt, &rttimer_id ) == -1 )
       return TIMER_ERROR_CREATE;

    return TIMER_ERROR_NONE;
}

//-----------------------------------------------------------------------------
// WAKEUP THREAD
//-----------------------------------------------------------------------------

// the reference to the wakeup thread
pthread_t wakeup_thread;

//-----------------------------------------------------------------------------

/// the wakeup thread function itself.  Established at the priority level one
/// below the controller (therefore two below coordinator), this thread is unblocked when
/// the controller blocks and therefore can detect when the controller thread is not
/// consuming budget but is not directly suspended by the coordinator.

// TODO : should send a timestamp instead as the time between notification and message
// arrival may be significant
void* wakeup( void* ) {
    char buf;
    timespec ts_sleep, ts_rem;
    ts_sleep.tv_sec = 0;
    ts_sleep.tv_nsec = 100;

    while( 1 ) {
        buf = 0;

        if( controller_is_blocked ) {
            //clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, time, NULL);
            //nanosleep( &ts_sleep, &ts_rem );  // DONT USE.  USE clock_nanosleep(...)
            continue;
        }

        // Note: may need mutex protection!
        controller_is_blocked = true;

        if( write( fd_wakeup_to_coordinator[1], &buf, 1 ) == -1) {
            std::string err_msg = "(coordinator.cpp) wakeup() failed making system call write(...)";
            error_log.write( error_string_bad_write( err_msg, errno ) );
            // TODO : determine if there is a need to bomb or recover
        }
    }
    return NULL;
}

//-----------------------------------------------------------------------------

/// Spawns the wakeup thread with a priority level one below the controller thread
/// (therefore two below the coordinator).  For more information see
/// wakeup( void* ) above
error_e create_wakeup_thread( void ) {

    pthread_attr_t attributes;
    struct sched_param param;

    pthread_attr_init( &attributes );

    // set an explicit schedule for the wakeup thread
    if( pthread_attr_setinheritsched( &attributes, PTHREAD_EXPLICIT_SCHED )  != 0 ) {
        sprintf( errstr, "(coordinator.cpp) create_wakeup_thread() failed calling pthread_attr_setinheritsched(...)\n" );
        error_log.write( errstr );
        return ERROR_FAILED;
    }

    // set schedule policy to round robin
    if( pthread_attr_setschedpolicy( &attributes, SCHED_RR ) != 0 ) {
        sprintf( errstr, "(coordinator.cpp) create_wakeup_thread() failed calling pthread_attr_setschedpolicy(...)\n" );
        error_log.write( errstr );
        return ERROR_FAILED;
    }

    // set schedule priority (2 below coordinator -> 1 below controller)
    param.sched_priority = coordinator_priority - 2;
    if( pthread_attr_setschedparam( &attributes, &param ) != 0 ) {
        sprintf( errstr, "(coordinator.cpp) create_wakeup_thread() failed calling pthread_attr_setschedparam(...)\n" );
        error_log.write( errstr );
        return ERROR_FAILED;
    }

    // create the wakeup thread
    if( pthread_create( &wakeup_thread, &attributes, wakeup, NULL ) != 0 ) {
        sprintf( errstr, "(coordinator.cpp) create_wakeup_thread() failed calling pthread_create(wakeup_thread,...)\n" );
        error_log.write( errstr );
        return ERROR_FAILED;
    }

    printf( "wakeup created\n" );

    // validation of the priority
    int policy;
    pthread_getschedparam( wakeup_thread, &policy, &param );
    printf( "wakeup priority: %d\n", param.sched_priority );

    pthread_attr_destroy( &attributes );

    return ERROR_NONE;
}

//-----------------------------------------------------------------------------

void init( int argc, char* argv[] ) {

    if( open_error_log( ) != ERROR_NONE ) {
        sprintf( errstr, "(coordinator.cpp) init() failed calling open_error_log().\nExiting\n" );
        printf( "%s", errstr );

        exit( 1 );
    }

    coordinator_pid = getpid( );

    //++++++++++++++++++++++++++++++++++++++++++++++++

    if( set_cpu( coordinator_pid, DEFAULT_PROCESSOR ) != ERROR_NONE ) {
        sprintf( errstr, "(coordinator.cpp) init() failed calling set_cpu(coordinator_pid,DEFAULT_PROCESSOR).\nExiting\n" );
        error_log.write( errstr );
        printf( "%s", errstr );

        error_log.close( );
        exit( 1 );
    }

    if( set_realtime_schedule_max( coordinator_pid, coordinator_priority ) != ERROR_NONE ) {
        sprintf( errstr, "(coordinator.cpp) init() failed calling set_realtime_schedule_max(coordinator_pid,coordinator_priority).\nExiting\n" );
        error_log.write( errstr );
        printf( "%s", errstr );

        error_log.close( );
        exit( 1 );
    }

    printf( "coordinator process priority: %d\n", coordinator_priority );

    //++++++++++++++++++++++++++++++++++++++++++++++++
    // Query the underlying system for information about the environment

    // Determine if the OS supports high resolution timers
    struct timespec res;
    clock_getres( CLOCK_MONOTONIC, &res );
    double clock_res = timespec_to_real( res );
    printf( "Clock resolution (secs): %10.9f\n", clock_res );

    if( res.tv_sec != 0 && res.tv_nsec != 1 ) {
        sprintf( errstr, "(coordinator.cpp) init() failed.  The host operating system does not support high resolution timers.  Consult your system documentation on enabling this feature.\nExiting\n" );
        error_log.write( errstr );
        printf( "%s", errstr );

        error_log.close( );
        exit( 1 );
    }

    // Get the cpu speed
    if( get_cpu_frequency( cpu_speed_hz, DEFAULT_PROCESSOR ) != CPUINFO_ERROR_NONE ) {
        sprintf( errstr, "(coordinator.cpp) init() failed calling read_cpuinfo()\nExiting\n" );
        error_log.write( errstr );
        printf( "%s", errstr );

        error_log.close( );
        exit( 1 );
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++
    // Initialize any shared components before initializing anything
    // else that might use them

    // Initialize IPC channels
    if( init_pipes( ) !=  ERROR_NONE ) {
        sprintf( errstr, "(coordinator.cpp) init() failed calling init_pipes()\nExiting\n" );
        error_log.write( errstr );
        printf( "%s", errstr );

        error_log.close( );
        exit( 1 );
    }

    // Create and initialize the actuator message buffer
    amsgbuffer = actuator_msg_buffer_c( ACTUATOR_MSG_BUFFER_NAME, ACTUATOR_MSG_BUFFER_MUTEX_NAME, true );
    if( amsgbuffer.open( ) != BUFFER_ERROR_NONE ) {
        sprintf( errstr, "(coordinator.cpp) init() failed calling actuator_msg_buffer_c.open(...,true)\n" );
        error_log.write( errstr );
        printf( "%s", errstr );

        close_pipes( );
        error_log.close( );
        exit( 1 );
    }

    //if( VERBOSE ) printf( "coordinator process initialized\n" );

    //++++++++++++++++++++++++++++++++++++++++++++++++
    // initialize dynamics

    if( init_dynamics( argc, argv ) != ERROR_NONE ) {
        sprintf( errstr, "(coordinator.cpp) init() failed calling init_dynamics(argc,argv)\nExiting\n" );
        error_log.write( errstr );
        printf( "%s", errstr );

        amsgbuffer.close( );
        close_pipes( );
        error_log.close( );
        exit( 1 );
    }


    //++++++++++++++++++++++++++++++++++++++++++++++++
    // initialize controller

    if( fork_controller( ) != ERROR_NONE ) {
        sprintf( errstr, "(coordinator.cpp) init() failed calling fork_controller()\nExiting\n" );
        error_log.write( errstr );
        printf( "%s", errstr );

        amsgbuffer.close( );
        close_pipes( );
        error_log.close( );
        exit( 1 );
    }
    CONTROLLER_PERIOD_NSEC = (unsigned long long)NSECS_PER_SEC / (unsigned long long)CONTROLLER_HZ;

    //++++++++++++++++++++++++++++++++++++++++++++++++
    // intialize all other monitor facilities

    printf( "(coordinator) setting initial timer\n" );

    if( create_hrrttimer( ) != TIMER_ERROR_NONE ) {
        sprintf( errstr, "(coordinator.cpp) init() failed calling create_hrrttimer()\nExiting\n" );
        error_log.write( errstr );
        printf( "%s", errstr );

        amsgbuffer.close( );
        close_pipes( );
        error_log.close( );
        exit( 1 );
    }

/*
    // initialize the wakeup thread
    if( create_wakeup_thread( ) != ERROR_NONE ) {
        amsgbuffer.close( );
        close_pipes( );
        error_log.close( );
        printf( "(coordinator.cpp) init() failed calling create_wakeup_thread()\nExiting\n" );
        exit( 1 );
    }
*/
    //++++++++++++++++++++++++++++++++++++++++++++++++
    // open any other logs

    /*
    // open the variance log
    if( open_variance_log( ) != ERROR_NONE ) {
        sprintf( errstr, "(coordinator.cpp) init() failed calling open_variance_log()\nExiting\n" );
        error_log.write( errstr );
        printf( "%s", errstr );

        amsgbuffer.close( );
        close_pipes( );
        error_log.close( );
        exit( 1 );
    }
    */

    timestamp_timer_buffer = timestamp_buffer_c( cpu_speed_hz, "timestamps_timer.dat" );
    if( timestamp_timer_buffer.open( ) != ERROR_NONE ) {
        sprintf( errstr, "(coordinator.cpp) init() failed calling timestamp_buffer_c.open()\nExiting\n" );
        error_log.write( errstr );
        printf( "%s", errstr );

        amsgbuffer.close( );
        close_pipes( );
        error_log.close( );
        exit( 1 );
    }
}

//-----------------------------------------------------------------------------

void shutdown( void ) {
    printf( "shutting down\n" );

    //variance_log.close( );

    //pthread_join( wakeup_thread, NULL );
    //pthread_kill( wakeup_thread, SIGKILL );

    shutdown_controller( );
    //resume_controller( );

    sleep( 1 );  // block for a second to allow controller to receive signal

    shutdown_dynamics( );

    amsgbuffer.close( );
    close_pipes( );

    timestamp_timer_buffer.close();

    // Note: can get a double free exception if the log is closed in this process
    // before it is closed in the controller

    error_log.close( );

    //exit( 0 );
}

//-----------------------------------------------------------------------------

Real previous_t = 0.0;
Real current_t = 0.0;
Real dt;

//-----------------------------------------------------------------------------

error_e service_controller_actuator_msg( void ) {
    actuator_msg_c msg;
    char buf = 0;

    if( amsgbuffer.read( msg ) != BUFFER_ERROR_NONE ) {
        sprintf( errstr, "(coordinator.cpp) main() failed calling actuator_msg_buffer_c.read(msg)\n" );
        error_log.write( errstr );
        return ERROR_FAILED;
    }

    switch( msg.header.type ) {
    case ACTUATOR_MSG_COMMAND:
        dynamics_get_command( );
        break;
    case ACTUATOR_MSG_REQUEST:
        // Note: the controller is blocking now on read from fd_coordinator_to_controller[0]

        if( !controller_fully_initialized ) {
            // this is a query for initial state
            controller_fully_initialized = true;
        } else {
            // otherwise progressive state

            // Note: right now this time is based on the frequency and cycle of the controller
            current_t = msg.state.time;
            dt = current_t - previous_t;
            previous_t = current_t;

            step_dynamics( dt );
        }

        dynamics_state_requested( );

        if( amsgbuffer.read( msg ) != BUFFER_ERROR_NONE)  {
            sprintf( errstr, "(coordinator.cpp) main() failed calling actuator_msg_buffer_c.read(msg) while servicing ACTUATOR_MSG_REQUEST\n" );
            error_log.write( errstr );
            return ERROR_FAILED;
        }

        if( write( fd_coordinator_to_controller[1], &buf, 1 ) == -1 ) {
            std::string err_msg = "(coordinator.cpp) main() failed making system call write(...) on fd_coordinator_to_controller[1]";
            error_log.write( error_string_bad_write( err_msg, errno ) );
            // TODO : determine if there is a need to recover
            return ERROR_FAILED;
        }
        break;
    case ACTUATOR_MSG_NO_COMMAND:
    case ACTUATOR_MSG_REPLY:
    case ACTUATOR_MSG_UNDEFINED:
        break;
    }
    return ERROR_NONE;
}


//-----------------------------------------------------------------------------
// ENTRY POINT
//-----------------------------------------------------------------------------

int main( int argc, char* argv[] ) {

    // Due to realtime scheduling, et al, this program must have root access to run.
    if( geteuid() != 0 ) {
        sprintf( errstr, "This program requires root access.  Re-run with sudo.\nExiting\n" );
        printf( "%s", errstr );
        exit( 1 );
    }

    char input_buffer;
    unsigned int timer_events = 0;
    unsigned long long ts_timer_armed, ts_timer_fired;

    controller_notification_c ctl_msg;
    fd_set fds_channels;
    int max_fd;

    init( argc, argv );

    max_fd = std::max( fd_timer_to_coordinator[0], fd_controller_to_coordinator[0] );
    //max_fd = std::max( max_fd, fd_controller_to_coordinator[0] );

    // lock into memory to prevent pagefaults
    mlockall( MCL_CURRENT );

    printf( "starting main loop\n" );

    while( 1 ) {

        FD_ZERO( &fds_channels );
        FD_SET( fd_timer_to_coordinator[0], &fds_channels );
        //FD_SET( fd_wakeup_to_coordinator[0], &fds_channels );
        FD_SET( fd_controller_to_coordinator[0], &fds_channels );

        if( select( max_fd + 1, &fds_channels, NULL, NULL, NULL ) ) {
            if( FD_ISSET( fd_timer_to_coordinator[0], &fds_channels ) ) {
                // timer event

                if( read( fd_timer_to_coordinator[0], &ts_timer_fired, sizeof(unsigned long long) ) == -1 ) {
                    std::string err_msg = "(coordinator.cpp) main() failed making system call read(...) on fd_timer_to_coordinator[0]";
                    error_log.write( error_string_bad_read( err_msg, errno ) );
                    // TODO : determine if there is a need to recover
                }

                timestamp_timer_buffer.write( ts_timer_fired - ts_timer_armed );

                if( ++timer_events == SIM_DURATION_CYCLES ) break;

                // controller needs to run!
                resume_controller( );

            } else if( FD_ISSET( fd_controller_to_coordinator[0], &fds_channels ) ) {
                // controller notification event

                if( read( fd_controller_to_coordinator[0], &ctl_msg, sizeof( controller_notification_c ) ) == -1 ) {
                    std::string err_msg = "(coordinator.cpp) main() failed making system call read(...) on fd_controller_to_coordinator[0]";
                    error_log.write( error_string_bad_read( err_msg, errno ) );
                    // TODO : determine if there is a need to recover
                }

                // check to see what kind of notification was sent.
                // if it's an actuator message then service it
                // else if it's a schedule request then schedule the controller timer
                if( ctl_msg.type == CONTROLLER_NOTIFICATION_ACTUATOR_EVENT ) {
                    if( service_controller_actuator_msg( ) != ERROR_NONE) {
                        sprintf( errstr, "(coordinator.cpp) main() failed calling service_controller_actuator_msg()\n" );
                        error_log.write( errstr );
                        break; //?
                    }
                } else if( ctl_msg.type == CONTROLLER_NOTIFICATION_SNOOZE ) {
                    // controller is preempted so it cannot run

                    // set a timer, suspend the controller until the timer fires
                    suspend_controller( );

                    if( arm_timer( ctl_msg.ts, ts_timer_armed ) != TIMER_ERROR_NONE ) {
                        sprintf( errstr, "(coordinator.cpp) main() failed calling arm_timer()\n" );
                        error_log.write( errstr );
                        break; //?
                    }

                    // possible to nanosleep here to optimize for a single controller; HOWEVER, is would interfere
                    // for multicontroller designs so the implementation of nanosleep is discouraged
                }
            } else if( FD_ISSET( fd_wakeup_to_coordinator[0], &fds_channels ) ) {
                // wakeup event

                if( read( fd_wakeup_to_coordinator[0], &input_buffer, 1 ) == -1 ) {
                    std::string err_msg = "(coordinator.cpp) main() failed making system call read(...) on fd_wakeup_to_coordinator[0]";
                    error_log.write( error_string_bad_read( err_msg, errno ) );
                    // TODO : determine if there is a need to recover
                }
            }
        }
    }

    munlockall();
    shutdown( );
}
