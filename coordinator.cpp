/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

coordinator.cpp
-----------------------------------------------------------------------------*/

#include <tas.h>
#include <dynamics_plugin.h>
#include <cpu.h>
#include <actuator.h>
#include <log.h>

//-----------------------------------------------------------------------------

using boost::dynamic_pointer_cast;

//-----------------------------------------------------------------------------

#define SIM_DURATION 10.0

//-----------------------------------------------------------------------------
// CPU
//-----------------------------------------------------------------------------

#define DEFAULT_PROCESSOR    0

//-----------------------------------------------------------------------------

cpuinfo_c cpuinfo;
double cpu_speed_mhz;
unsigned long long cpu_speed_hz;

//-----------------------------------------------------------------------------
// LOGGING
//-----------------------------------------------------------------------------

log_c variance_log;
log_c error_log;
char strbuffer[ 256 ];

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
    dup2( error_log.get_fd(), FD_ERROR_LOG );

    return ERROR_NONE;
}

//-----------------------------------------------------------------------------

error_e open_variance_log( void ) {
    variance_log = log_c( "variance.dat" );
    //variance_log = log_c( LOG_CHANNEL_STDOUT );

    if( variance_log.open( ) != LOG_ERROR_NONE ) {
        sprintf( strbuffer, "(coordinator.cpp) open_variance_log() failed calling log_c.open() on file variance.dat\n" );
        error_log.write( strbuffer );
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
  invokes the intialization function (init_fun_t) of all registered dynamics plugins by
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
  invokes the run function (run_fun_t) of all the registered dynamics plugins
*/
void run_dynamics( Real dt ) {
    if(VERBOSE) printf( "(coordinator) Running Dynamics\n" );

    for( std::list< dynamics_plugin_c >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it->run)( dt );
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
    if(VERBOSE) printf( "(coordinator) Requesting Dynamics State\n" );

    for( std::list< dynamics_plugin_c >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it->write_state)( );
    }
}

//-----------------------------------------------------------------------------

void dynamics_get_command( void ) {
    if(VERBOSE) printf( "(coordinator) Getting Command\n" );

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
    if( VERBOSE ) printf("(coordinator) dynamics initialized\n");

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
        sprintf( strbuffer, "(coordinator.cpp) init_pipes() failed calling pipe(fd_timer_to_coordinator)\n" );
        error_log.write( strbuffer );
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
        sprintf( strbuffer, "(coordinator.cpp) init_pipes() failed calling pipe(fd_wakeup_to_coordinator)\n" );
        error_log.write( strbuffer );
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
        sprintf( strbuffer, "(coordinator.cpp) init_pipes() failed calling pipe(fd_coordinator_to_controller)\n" );
        error_log.write( strbuffer );
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
        sprintf( strbuffer, "(coordinator.cpp) init_pipes() failed calling pipe(fd_controller_to_coordinator)\n" );
        error_log.write( strbuffer );
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

// Coordinator process information
static pid_t coordinator_pid;
static int coordinator_priority;

// Controller process information
static int controller_pid;

bool controller_is_blocked;
bool controller_is_suspended;

bool controller_fully_initialized = false;
//-----------------------------------------------------------------------------

/// Creates the controller process with a priority level one step below the coordinator.
/// Note: an ERROR_NONE in the coordinator does not really guarantee that the controller
/// started properly
error_e fork_controller( void ) {
    cpu_set_t cpuset_mask;
    struct sched_param param;

    controller_pid = fork( );
    // handle any fork error
    if( controller_pid < 0 ) {
        sprintf( strbuffer, "(coordinator.cpp) fork_controller() failed calling fork()\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }

    // start of the contoller process
    if( controller_pid == 0 ) {

        controller_pid = getpid( );

        //+++++++++++++++++++++++++++++++++
        // restrict the cpu set s.t. controller only runs on a single processor

        // zero out the cpu set
        CPU_ZERO( &cpuset_mask );
        // set the cpu set s.t. controller only runs on 1 processor specified by DEFAULT_PROCESSOR
        CPU_SET( DEFAULT_PROCESSOR, &cpuset_mask );
        if( sched_setaffinity( controller_pid, sizeof(cpuset_mask), &cpuset_mask ) == -1 ) {
            // there was an error setting the affinity for the controller
            // NOTE: can check errno if this occurs
            sprintf( strbuffer, "(coordinator.cpp) fork_controller() failed calling sched_setaffinity(controller_pid, ...)\n" );
            error_log.write( strbuffer );
            return ERROR_FAILED;
        }
        /*
        // testing sanity check ... TO BE COMMENTED
        int ret = sched_getaffinity( controller_pid, sizeof(cpuset_mask), &cpuset_mask );
        printf( " sched_getaffinity = %d, cpuset_mask = %08lx\n", sizeof(cpuset_mask), cpuset_mask );
        */

        //+++++++++++++++++++++++++++++++++
        // set the controller schedule policy

        // query the current policy
        int sched_policy = sched_getscheduler( controller_pid );
        if( sched_policy == -1 ) {
            // there was an error getting the scheduler policy for the controller
            // NOTE: can check errno if this occurs
            sprintf( strbuffer, "(coordinator.cpp) fork_controller() failed calling sched_getschduler(controller_pid)\n" );
            error_log.write( strbuffer );
            return ERROR_FAILED;
        }

        // adjust the priority to one below the coordinator's
        param.sched_priority = coordinator_priority - 1;

        // set the new policy
        // NOTE: Round Robin is opted for in this case.  FIFO an option.
        // man pages claim that either of these policies are real-time
        sched_setscheduler( controller_pid, SCHED_RR, &param );

        // validate the priority
        sched_getparam( controller_pid, &param );
        printf( "controller process priority: %d\n", param.sched_priority );

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
        execl( DEFAULT_CONTROLLER_PROGRAM, "controller", NULL );

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

// timer variables
//clockid_t rttimer_clock;
sigset_t rttimer_mask;

// The signal action to reference controller's process signal handler
struct sigaction rttimer_sigaction;

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

/// Creates a real-time timer to monitor the controller process
timer_err_e create_rttimer( void ) {
    timer_t rttimer_id;
    struct sigevent sevt;
    struct itimerspec its;
    clockid_t rttimer_clock;

    // Establish handler for timer signal
    rttimer_sigaction.sa_flags = SA_SIGINFO;
    rttimer_sigaction.sa_sigaction = rttimer_sighandler;
    sigemptyset( &rttimer_sigaction.sa_mask );
    if( sigaction( RTTIMER_SIGNAL, &rttimer_sigaction, NULL ) == -1 )
        return TIMER_ERROR_SIGACTION;

    // Intialize the signal mask
    sigemptyset( &rttimer_mask );
    sigaddset( &rttimer_mask, RTTIMER_SIGNAL );

    // Block timer signal temporarily
    if( block_rttimer() != TIMER_ERROR_NONE )
        return TIMER_ERROR_SIGPROCMASK;

    // Query the clock id for the controller process
    if( clock_getcpuclockid( controller_pid, &rttimer_clock ) != 0 )
        return TIMER_ERROR_GETCLOCKID;

    // Create the timer
    sevt.sigev_notify = SIGEV_SIGNAL;
    sevt.sigev_signo = RTTIMER_SIGNAL;
    sevt.sigev_value.sival_ptr = &rttimer_id;
    if( timer_create( rttimer_clock, &sevt, &rttimer_id ) == -1 )
        //if( timer_create( CLOCK_MONOTONIC, &sevt, &rttimer_id ) == -1 )
        return TIMER_ERROR_CREATE;

    // intial delay for the timer
    its.it_value.tv_sec = DEFAULT_INITDELAY_RTTIMER_SEC;
    its.it_value.tv_nsec = DEFAULT_INITDELAY_RTTIMER_NSEC;
    // budget for the controller
    its.it_interval.tv_sec = DEFAULT_BUDGET_RTTIMER_SEC;
    its.it_interval.tv_nsec = DEFAULT_BUDGET_RTTIMER_NSEC;

    // Set up the timer
    if( timer_settime( rttimer_id, 0, &its, NULL ) == -1 )
        return TIMER_ERROR_SETTIME;

    // NOTE: The timer is blocked at this point.  It must be unblocked in
    // the coordinator process for the timer to function

    return TIMER_ERROR_NONE;
}

//-----------------------------------------------------------------------------

timer_t rttimer_id;

//-----------------------------------------------------------------------------

timer_err_e arm_timer( unsigned long long& ts ) {

    struct itimerspec its;

    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 0;
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = 1000000;
    //its.it_value.tv_nsec = 800000;

    // timestamp here?
    rdtscll( ts );

    //if( timer_settime( rttimer_id, TIMER_ABSTIME, &its, NULL ) == -1 )
    if( timer_settime( rttimer_id, 0, &its, NULL ) == -1 )
        return TIMER_ERROR_SETTIME;

    // or timestamp here?
    //rdtscll( ts );


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
/*
/// Initializes the controller monitoring timer which determines when the controller has
/// exhausted its budget
error_e init_rttimer( void ) {

    if( create_hrrttimer( ) != TIMER_ERROR_NONE ) {
        sprintf( strbuffer, "(coordinator.cpp) init_rttimer() failed calling create_hrrttimer()\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }

    if( VERBOSE ) printf("(coordinator) rttimer initialized\n");

    // unblock the controller process
    resume_controller( );
    // Note: this doesn't mean the controller is fully initialized

    return ERROR_NONE;
}
*/
//-----------------------------------------------------------------------------

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
        sprintf( strbuffer, "(coordinator.cpp) create_wakeup_thread() failed calling pthread_attr_setinheritsched(...)\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }

    // set schedule policy to round robin
    if( pthread_attr_setschedpolicy( &attributes, SCHED_RR ) != 0 ) {
        sprintf( strbuffer, "(coordinator.cpp) create_wakeup_thread() failed calling pthread_attr_setschedpolicy(...)\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }

    // set schedule priority (2 below coordinator -> 1 below controller)
    param.sched_priority = coordinator_priority - 2;
    if( pthread_attr_setschedparam( &attributes, &param ) != 0 ) {
        sprintf( strbuffer, "(coordinator.cpp) create_wakeup_thread() failed calling pthread_attr_setschedparam(...)\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }

    // create the wakeup thread
    if( pthread_create( &wakeup_thread, &attributes, wakeup, NULL ) != 0 ) {
        sprintf( strbuffer, "(coordinator.cpp) create_wakeup_thread() failed calling pthread_create(wakeup_thread,...)\n" );
        error_log.write( strbuffer );
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

error_e set_schedule( void ) {
    // restrict the cpu set s.t. controller only runs on a single processor
    cpu_set_t cpuset_mask;
    // zero out the cpu set
    CPU_ZERO( &cpuset_mask );
    // set the cpu set s.t. controller only runs on 1 processor specified by DEFAULT_PROCESSOR
    CPU_SET( DEFAULT_PROCESSOR, &cpuset_mask );

    if ( sched_setaffinity( 0, sizeof(cpuset_mask), &cpuset_mask ) == -1 ) {
        // there was an error setting the affinity for the coordinator
        // NOTE: can check errno if this occurs
        sprintf( strbuffer, "(coordinator.cpp) set_schedule() failed calling sched_setaffinity(coordinator_pid,...)\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }

    /*
    // testing sanity check ... TO BE COMMENTED
    int ret = sched_getaffinity( 0, sizeof(cpuset_mask), &cpuset_mask );
    printf( " sched_getaffinity = %d, cpuset_mask = %08lx\n", sizeof(cpuset_mask), cpuset_mask );
    */

    // set the scheduling policy and the priority where priority should be the
    // highest possible priority, i.e. min, for the round robin scheduler
    struct sched_param param;
    param.sched_priority = sched_get_priority_max( SCHED_RR );
    sched_setscheduler( 0, SCHED_RR, &param );

    // validate the scheduling policy
    int policy = sched_getscheduler( 0 );
    if( policy != SCHED_RR ) {
        sprintf( strbuffer, "(coordinator.cpp) set_schedule() failed setting SCHED_RR policy.  The actual policy is %s\n",
                 (policy == SCHED_OTHER) ? "SCHED_OTHER" :
                                           (policy == SCHED_FIFO) ? "SCHED_FIFO" :
                                                                    "UNDEFINED" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }

    // validate the priority
    sched_getparam( 0, &param );
    coordinator_priority = param.sched_priority;
    printf( "coordinator process priority: %d\n", coordinator_priority );

    return ERROR_NONE;
}

//-----------------------------------------------------------------------------

error_e read_cpuinfo( void ) {
    if( cpuinfo.load( ) != CPUINFO_ERROR_NONE ) {
        sprintf( strbuffer, "(coordinator.cpp) read_cpuinfo() failed calling cpuinfo_c.load()\n" );
        error_log.write( strbuffer );
        return ERROR_FAILED;
    }
    printf( "cpuinfo loaded\n" );

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
            sprintf( strbuffer, "(coordinator.cpp) read_cpuinfo() failed to find default processor in cpus.\n" );
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

void init( int argc, char* argv[] ) {

    if( open_error_log( ) != ERROR_NONE ) {
        exit( 1 );
    }

    coordinator_pid = getpid( );

    //++++++++++++++++++++++++++++++++++++++++++++++++

    // Set the schedule for the coordinator itself
    if( set_schedule( ) != ERROR_NONE ) {
        error_log.close( );
        printf( "(coordinator.cpp) init() failed calling set_schedule()\nExiting\n" );
        exit( 1 );
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++
    // Query the underlying system for information about the environment

    // Determine if the OS supports high resolution timers
    struct timespec res;
    clock_getres( CLOCK_MONOTONIC, &res );
    double clock_res = timespec_to_real( res );
    printf( "Clock resolution (secs): %10.9f\n", clock_res );

    if( res.tv_sec != 0 && res.tv_nsec != 1 ) {
        sprintf( strbuffer, "(coordinator.cpp) init() failed.  The host operating system does not support high resolution timers.  Consult your system documentation on enabling this feature.\n" );
        error_log.write( strbuffer );
        error_log.close( );
        printf( "%s\nExiting\n", strbuffer );
        exit( 1 );
    }

    // Get the cpu speed
    if( read_cpuinfo( ) != ERROR_NONE ) {
        error_log.close( );
        printf( "(coordinator.cpp) init() failed calling read_cpuinfo()\nExiting\n" );
        exit( 1 );
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++
    // Initialize any shared components before initializing anything
    // else that might use them

    // Initialize IPC channels
    if( init_pipes( ) !=  ERROR_NONE ) {
        error_log.close( );
        printf( "(coordinator.cpp) init() failed calling init_pipes()\nExiting\n" );
        exit( 1 );
    }

    // Create and initialize the actuator message buffer
    amsgbuffer = actuator_msg_buffer_c( ACTUATOR_MSG_BUFFER_NAME, ACTUATOR_MSG_BUFFER_MUTEX_NAME, true );
    if( amsgbuffer.open( ) != BUFFER_ERROR_NONE ) {
        sprintf( strbuffer, "(coordinator.cpp) init() failed calling actuator_msg_buffer_c.open(...,true)\n" );
        error_log.write( strbuffer );
        printf( "%s", strbuffer );

        close_pipes( );
        error_log.close( );
        exit( 1 );
    }

    //if( VERBOSE ) printf( "coordinator process initialized\n" );

    //++++++++++++++++++++++++++++++++++++++++++++++++
    // initialize dynamics

    if( init_dynamics( argc, argv ) != ERROR_NONE ) {
        sprintf( strbuffer, "(coordinator.cpp) init() failed calling init_dynamics(argc,argv)\nExiting\n" );
        error_log.write( strbuffer );
        printf( "%s", strbuffer );

        amsgbuffer.close( );
        close_pipes( );
        error_log.close( );
        exit( 1 );
    }


    //++++++++++++++++++++++++++++++++++++++++++++++++
    // initialize controller

    if( fork_controller( ) != ERROR_NONE ) {
        sprintf( strbuffer, "(coordinator.cpp) init() failed calling fork_controller()\nExiting\n" );
        error_log.write( strbuffer );
        printf( "%s", strbuffer );

        amsgbuffer.close( );
        close_pipes( );
        error_log.close( );
        exit( 1 );
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++
    // intialize all other monitor facilities

    printf( "(coordinator) setting initial timer\n" );

    if( create_hrrttimer( ) != TIMER_ERROR_NONE ) {
        sprintf( strbuffer, "(coordinator.cpp) init() failed calling create_hrrttimer()\nExiting\n" );
        error_log.write( strbuffer );
        printf( "%s", strbuffer );

        amsgbuffer.close( );
        close_pipes( );
        error_log.close( );
        exit( 1 );
    }

    /*
    // initialize the timer
    if( init_rttimer( ) != ERROR_NONE ) {
        amsgbuffer.close( );
        close_pipes( );
        error_log.close( );
        printf( "(coordinator.cpp) init() failed calling init_rttimer()\nExiting\n" );
        exit( 1 );
    }
    */
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

    // open the variance log
    if( open_variance_log( ) != ERROR_NONE ) {
        sprintf( strbuffer, "(coordinator.cpp) init() failed calling open_variance_log()\nExiting\n" );
        error_log.write( strbuffer );
        printf( "%s", strbuffer );

        amsgbuffer.close( );
        close_pipes( );
        error_log.close( );
        exit( 1 );
    }
}

//-----------------------------------------------------------------------------

void shutdown( void ) {
    variance_log.close( );

    //pthread_join( wakeup_thread, NULL );
    pthread_kill( wakeup_thread, SIGKILL );

    shutdown_controller( );
    //resume_controller( );

    sleep( 1 );  // block for a second to allow controller to receive signal

    amsgbuffer.close( );
    close_pipes( );

    // Note: can get a double free exception if the log is closed in this process
    // before it is closed in the controller
    error_log.close( );

    //exit( 0 );
}

Real previous_t = 0.0;
Real current_t = 0.0;
Real dt;

void service_controller_actuator_msg( void ) {
    actuator_msg_c msg;
    if( amsgbuffer.read( msg ) != BUFFER_ERROR_NONE ) {
        sprintf( strbuffer, "(coordinator.cpp) main() failed calling actuator_msg_buffer_c.read(msg)\n" );
        error_log.write( strbuffer );
    }

    if(VERBOSE) msg.print();

    char buf = 0;

    switch( msg.header.type ) {
    case ACTUATOR_MSG_COMMAND:
        if(VERBOSE) printf( "(coordinator) received ACTUATOR_MSG_COMMAND\n" );

        //suspend_controller( );

        dynamics_get_command( );

        //resume_controller( );
        break;
    case ACTUATOR_MSG_REQUEST:
        if(VERBOSE) printf( "(coordinator) received ACTUATOR_MSG_REQUEST\n" );

        // Note: the controller is blocking now on read from fd_coordinator_to_controller[0]

        if( !controller_fully_initialized ) {
            // indicates that the controller is fully initialized and timing can begin
            // and that this is querying for initial state
            controller_fully_initialized = true;

            // therefore suspend the controller and unblock the timer
            //coordinator_suspend_controller( );    // be careful.  If you suspend must resume
            //coordinator_unblock_controllers_signal_rttimer( );

            dynamics_state_requested( );

            if( amsgbuffer.read( msg ) != BUFFER_ERROR_NONE)  {
                sprintf( strbuffer, "(coordinator.cpp) main() failed calling actuator_msg_buffer_c.read(msg) while servicing initial ACTUATOR_MSG_REQUEST\n" );
                error_log.write( strbuffer );
            }

            if(VERBOSE) msg.print();

            if(VERBOSE) printf( "(coordinator) notifying controller state available\n" );

            //coordinator_resume_controller( );  // paired with above suspend
            if( write( fd_coordinator_to_controller[1], &buf, 1 ) == -1 ) {
                std::string err_msg = "(coordinator.cpp) main() failed making system call write(...) on fd_coordinator_to_controller[1]";
                error_log.write( error_string_bad_write( err_msg, errno ) );
                // TODO : determine if there is a need to recover
            }
        } else {

            // right now this time is based on the frequency and cycle of the controller
            current_t = msg.state.time;

            dt = current_t - previous_t;
            //printf( "dt: %e\n", dt );
            printf( "dt: %10.9f\n", dt );

            run_dynamics( dt );

            dynamics_state_requested( );

            if( amsgbuffer.read( msg ) != BUFFER_ERROR_NONE)  {
                sprintf( strbuffer, "(coordinator.cpp) main() failed calling actuator_msg_buffer_c.read(msg) while servicing ACTUATOR_MSG_REQUEST\n" );
                error_log.write( strbuffer );
            }

            if(VERBOSE) msg.print();

            if(VERBOSE) printf( "(coordinator) notifying controller state available\n" );

            //write( fd_coordinator_to_controller[1], &buf, 1 );
            if( write( fd_coordinator_to_controller[1], &buf, 1 ) == -1 ) {
                std::string err_msg = "(coordinator.cpp) main() failed making system call write(...) on fd_coordinator_to_controller[1]";
                error_log.write( error_string_bad_write( err_msg, errno ) );
                // TODO : determine if there is a need to recover
            }

            previous_t = current_t;
            // ]

            // ++++++++++++++++

            // if dynamics ahead of controller ?? [
            // use the current state?

            // ]

            // ++++++++++++++++

            // if dynamics in step with controller ?? [
            // use the current state?

            // ]
        }
        break;
    case ACTUATOR_MSG_NO_COMMAND:
    case ACTUATOR_MSG_REPLY:
    case ACTUATOR_MSG_UNDEFINED:
        break;
    }
}

//-----------------------------------------------------------------------------
// ENTRY POINT
//-----------------------------------------------------------------------------

int main( int argc, char* argv[] ) {

    // Due to realtime scheduling, et al, this program must have root access to run.
    if( geteuid() != 0 ) {
        sprintf( strbuffer, "This program requires root access.  Re-run with sudo.\nExiting\n" );
        printf( "%s", strbuffer );
        exit( 1 );
    }

    init( argc, argv );

    char input_buffer;
    unsigned long long ts;
    Real t;

    unsigned long long ts_timer_armed, ts_timer_fired, dts_timer;
    Real dt_timer;

    int max_fd = std::max( fd_timer_to_coordinator[0], fd_wakeup_to_coordinator[0] );
    max_fd = std::max( max_fd, fd_controller_to_coordinator[0] );

    controller_notification_c ctl_msg;

    // lock into memory to prevent pagefaults
    mlockall( MCL_CURRENT );

    printf( "starting main loop\n" );

    while( 1 ) {

        //if( current_t > SIM_DURATION ) break;

        fd_set fds_channels;

        FD_ZERO( &fds_channels );
        FD_SET( fd_timer_to_coordinator[0], &fds_channels );
        FD_SET( fd_wakeup_to_coordinator[0], &fds_channels );
        FD_SET( fd_controller_to_coordinator[0], &fds_channels );

        // TODO: review the following code
        int result = select( max_fd + 1, &fds_channels, NULL, NULL, NULL );
        if( result ) {
            if( FD_ISSET( fd_timer_to_coordinator[0], &fds_channels ) ) {

                //printf( "timer event\n" );

                if( read( fd_timer_to_coordinator[0], &ts_timer_fired, sizeof(unsigned long long) ) == -1 ) {
                    std::string err_msg = "(coordinator.cpp) main() failed making system call read(...) on fd_timer_to_coordinator[0]";
                    error_log.write( error_string_bad_read( err_msg, errno ) );
                    // TODO : determine if there is a need to recover
                }

                dts_timer = ts_timer_fired - ts_timer_armed;
                dt_timer = cycles_to_seconds( dts_timer, cpu_speed_hz );

                printf( "(coordinator) Timer Interval (cycles, seconds): %lld, %11.10f\n", dts_timer, dt_timer );

                if( dt_timer > 0.001 * 1.20 ) break;

                // controller needs to run!
                resume_controller( );

                if( arm_timer( ts_timer_armed ) != TIMER_ERROR_NONE ) {
                    sprintf( strbuffer, "(coordinator.cpp) main() failed calling arm_timer()\n" );
                    error_log.write( strbuffer );
                    return ERROR_FAILED;
                }
            } else if( FD_ISSET( fd_controller_to_coordinator[0], &fds_channels ) ) {
                //printf( "(coordinator) received a notification from controller\n" );

                // controller notification event
                if( read( fd_controller_to_coordinator[0], &ctl_msg, sizeof( controller_notification_c ) ) == -1 ) {
                    std::string err_msg = "(coordinator.cpp) main() failed making system call read(...) on fd_controller_to_coordinator[0]";
                    error_log.write( error_string_bad_read( err_msg, errno ) );
                    // TODO : determine if there is a need to recover
                }


                //if( VERBOSE ) printf( "(coordinator) received a notification from controller\n" );
                // now check the message buffer and respond to the type of message

                // check to see what kind of notification was sent.
                // if it's an actuator message then service it
                // else if it's a schedule request then schedule the controller timer
                if( ctl_msg.type == CONTROLLER_NOTIFICATION_ACTUATOR_EVENT ) {
                    service_controller_actuator_msg( );
                } else if( ctl_msg.type == CONTROLLER_NOTIFICATION_SNOOZE ) {


                    printf( "(coordinator) received a snooze notification from controller\n" );

                    // set a timer, suspend the controller until the timer fires
                    suspend_controller( );

                    printf( "(coordinator) setting timer\n" );

                    if( arm_timer( ts_timer_armed ) != TIMER_ERROR_NONE ) {
                        sprintf( strbuffer, "(coordinator.cpp) main() failed calling arm_timer()\n" );
                        error_log.write( strbuffer );
                        return ERROR_FAILED;
                    }
                }

            } else if( FD_ISSET( fd_wakeup_to_coordinator[0], &fds_channels ) ) {
                printf( "wakeup event\n" );

                // What if the controller is suspended?
                // Wakeup doesn't appear to happen because suspension is in main thread and resumption occurs before main thread is blocked


                // wakeup event
                if( read( fd_wakeup_to_coordinator[0], &input_buffer, 1 ) == -1 ) {
                    std::string err_msg = "(coordinator.cpp) main() failed making system call read(...) on fd_wakeup_to_coordinator[0]";
                    error_log.write( error_string_bad_read( err_msg, errno ) );
                    // TODO : determine if there is a need to recover
                }

                printf( "(coordinator) received a wakeup from a blocked controller\n" );
            }
        }
    }
    munlockall();
    shutdown( );
}
