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
// CPU
//-----------------------------------------------------------------------------

#define DEFAULT_PROCESSOR    0

//-----------------------------------------------------------------------------

cpuinfo_c cpuinfo;
double cpu_speed_mhz;
unsigned long long cpu_speed_hz;

//-----------------------------------------------------------------------------

double cycles_to_seconds( const unsigned long long& cycles ) {
    return (double)cycles / (double)cpu_speed_hz;
} 

//-----------------------------------------------------------------------------
// LOGGING
//-----------------------------------------------------------------------------

log_c variance_log;
log_c error_log;
char strbuffer[ 256 ];

//-----------------------------------------------------------------------------

int open_error_log( void ) {
    error_log = log_c( LOG_CHANNEL_FILE );

    if( error_log.open( "error.log" ) != LOG_ERROR_NONE ) {
	//sprintf( strbuffer, "ERROR: opening log file: error.log" );
	printf( "ERROR: opening log file: error.log" );
	return 1;
    }
    return 0;
}

//-----------------------------------------------------------------------------

int open_variance_log( void ) {
    //variance_log = log_c( LOG_CHANNEL_FILE );

    if( variance_log.open( "variance.dat" ) != LOG_ERROR_NONE ) {
	sprintf( strbuffer, "ERROR: opening output file: variance.dat" );
	error_log.write( strbuffer );
	return 1;
    }
    return 0;
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

void init_dynamics( const int& argc, char* argv[] ) {

    dynamics_plugin_c plugin;
    int result = plugin.read( DYNAMICS_PLUGIN );	// TODO: Sanity/Safety checking
    plugin.init( argc, argv );

    // if both functions were located then push the pair into the list of plugins
    if( !result ) plugins.push_back( plugin );
    if( VERBOSE ) printf("(coordinator) dynamics initialized\n");
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
void initialize_pipes( void ) {
    int flags;

    // NOTE: a read channel needs to be blocking to force  block waiting for event
    // NOTE: a write channel should be non-blocking (won't ever fill up buffer anyway)

    // Open the timer to coordinator channel for timer events
    if( pipe( fd_timer_to_coordinator ) != 0 ) {
        throw std::runtime_error( "Failed to open fd_timer_to_coordinator pipe." ) ;
    }
    flags = fcntl( fd_timer_to_coordinator[0], F_GETFL, 0 );
    fcntl( fd_timer_to_coordinator[0], F_SETFL, flags );
    flags = fcntl( fd_timer_to_coordinator[1], F_GETFL, 0 );
    fcntl( fd_timer_to_coordinator[1], F_SETFL, flags | O_NONBLOCK );

    // Open the wakeup to coordinator channel for wakeup (blocking) events
    if( pipe( fd_wakeup_to_coordinator ) != 0 ) {
        close( fd_timer_to_coordinator[0] );
        close( fd_timer_to_coordinator[1] );
        throw std::runtime_error( "Failed to open fd_wakeup_to_coordinator pipe." ) ;
    }
    flags = fcntl( fd_wakeup_to_coordinator[0], F_GETFL, 0 );
    fcntl( fd_wakeup_to_coordinator[0], F_SETFL, flags );
    flags = fcntl( fd_wakeup_to_coordinator[1], F_GETFL, 0 );
    fcntl( fd_wakeup_to_coordinator[1], F_SETFL, flags | O_NONBLOCK );

    // Open the coordinator to controller channel for notifications
    if( pipe( fd_coordinator_to_controller ) != 0 ) {
        close( fd_timer_to_coordinator[0] );
        close( fd_timer_to_coordinator[1] );
        close( fd_wakeup_to_coordinator[0] );
        close( fd_wakeup_to_coordinator[1] );
        throw std::runtime_error( "Failed to open fd_coordinator_to_controller pipe." ) ;
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
        throw std::runtime_error( "Failed to open fd_controller_to_coordinator pipe." ) ;
    }
    flags = fcntl( fd_controller_to_coordinator[0], F_GETFL, 0 );
    fcntl( fd_controller_to_coordinator[0], F_SETFL, flags );
    flags = fcntl( fd_controller_to_coordinator[1], F_GETFL, 0 );
    fcntl( fd_controller_to_coordinator[1], F_SETFL, flags | O_NONBLOCK );

}

//-----------------------------------------------------------------------------
// PROCESS MANAGEMENT
//-----------------------------------------------------------------------------

// Coordinator process information
static pid_t coordinator_pid;
static int coordinator_priority;

// Controller process information
static int controller_pid;

bool controller_fully_initialized = false;
//-----------------------------------------------------------------------------

/// Creates the controller process with a priority level one step below the coordinator.
void fork_controller( void ) {

    controller_pid = fork( );
    if( controller_pid < 0 ) {
        throw std::runtime_error( "Failed to fork controller." ) ;
    }
    if( controller_pid == 0 ) {

        controller_pid = getpid( );

        // restrict the cpu set s.t. controller only runs on a single processor
        cpu_set_t cpuset_mask;
        // zero out the cpu set
        CPU_ZERO( &cpuset_mask );
        // set the cpu set s.t. controller only runs on 1 processor specified by DEFAULT_PROCESSOR
        CPU_SET( DEFAULT_PROCESSOR, &cpuset_mask );
        if( sched_setaffinity( controller_pid, sizeof(cpuset_mask), &cpuset_mask ) == -1 ) {
            // there was an error setting the affinity for the controller
            // NOTE: can check errno if this occurs
            printf( "ERROR: Failed to set affinity for controller process.\n" );
            // could throw or exit or perror
        }

        int sched_policy = sched_getscheduler( controller_pid );
        if( sched_policy == -1 ) {
            // there was an error getting the scheduler policy for the controller
            // NOTE: can check errno if this occurs
            printf( "ERROR: Failed to get scheduler policy for controller process.\n" );
            // could throw or exit or perror
        } else {
            struct sched_param param;
            param.sched_priority = coordinator_priority - 1;
            // set the policy
            // NOTE: Round Robin is opted for in this case.  FIFO an option.
            // man pages claim that either of these policies are real-time
            sched_setscheduler( controller_pid, SCHED_RR, &param );

            sched_getparam( controller_pid, &param );
            printf( "controller process priority: %d\n", param.sched_priority );
        }

        /*
        // testing sanity check ... TO BE COMMENTED
        int ret = sched_getaffinity( controller_pid, sizeof(cpuset_mask), &cpuset_mask );
        printf( " sched_getaffinity = %d, cpuset_mask = %08lx\n", sizeof(cpuset_mask), cpuset_mask );
        */

        // duplicate the default fd's into known fd's so that the controller explicitly knows
        // what the channels are.
        // Note: dup2 explicitly closes the channel you are duping into so there is no need
        // for a seperate close call preceding the dup2 call unless some kernel programmer
        // didn't meet the spec.
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
        execl( DEFAULT_CONTROLLER_PROGRAM, "controller", 0 );

        // exit on fail to exec.  Can only get here if execl fails.
        _exit( 1 );
    }
}

//-----------------------------------------------------------------------------

/// Pause the controller from the coordinator process
void suspend_controller( void ) {

    kill( controller_pid, SIGSTOP );
}

//-----------------------------------------------------------------------------

/// Continue the controller from the coordinator process
void resume_controller( void ) {

    kill( controller_pid, SIGCONT );
}

//-----------------------------------------------------------------------------

void shutdown_controller( void ) {

    //kill( controller_pid, SIGTERM );
    kill( controller_pid, SIGKILL );
}

//-----------------------------------------------------------------------------
// TIMER CONTROL
//-----------------------------------------------------------------------------
/*
// Time conversions if necessary
#define NSECS_PER_SEC 1E9
#define USECS_PER_SEC 1E6

// Time budget for controller process
#define DEFAULT_BUDGET_RTTIMER_NSEC             1E8         // 0.1 second
#define DEFAULT_BUDGET_RTTIMER_SEC              0           // 0 second

// Initial delay for timer
// NOTE: There must be some delay otherwise timer is disarmed
#define DEFAULT_INITDELAY_RTTIMER_NSEC          1           // 1 nanoseconds
#define DEFAULT_INITDELAY_RTTIMER_SEC           0           // 0 second
*/
//-----------------------------------------------------------------------------
// Error codes for timer control

enum timer_err_e {
    TIMER_ERROR_NONE = 0,
    TIMER_ERROR_SIGACTION,
    TIMER_ERROR_SIGPROCMASK,
    TIMER_ERROR_CREATE,
    TIMER_ERROR_SETTIME,
    TIMER_ERROR_GETCLOCKID
};
//-----------------------------------------------------------------------------

// The signal identifier for the controller's real-time timer
#define RTTIMER_SIGNAL                          SIGRTMIN + 4

// timer variables
clockid_t rttimer_clock;
sigset_t rttimer_mask;

// The signal action to reference controller's process signal handler
struct sigaction rttimer_sigaction;

//-----------------------------------------------------------------------------

/// The signal handler for the controller's timer.  Is called only when the controller's
/// budget is exhausted.  Puts a message out onto the comm channel the coordinator
/// is listening to for notifications about this event
void rttimer_sighandler( int signum, siginfo_t *si, void *data ) {
    if( VERBOSE ) printf( "(timer) event\n" );
    //char buf = 0;
    //write( fd_timer_to_coordinator[1], &buf, 1 );
    unsigned long long ts;
    rdtscll( ts );
    write( fd_timer_to_coordinator[1], &ts, sizeof(unsigned long long) );
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

/// Create the timer to monitor the controller process
timer_err_e create_rttimer( void ) {
    timer_t rttimer_id;
    struct sigevent sevt;
    struct itimerspec its;

    // Establish handler for timer signal
    rttimer_sigaction.sa_flags = SA_SIGINFO;
    rttimer_sigaction.sa_sigaction = rttimer_sighandler;
    sigemptyset( &rttimer_sigaction.sa_mask );
    if( sigaction( RTTIMER_SIGNAL, &rttimer_sigaction, NULL ) == -1 )
       return TIMER_ERROR_SIGACTION;

    // intialize the signal mask
    sigemptyset( &rttimer_mask );
    sigaddset( &rttimer_mask, RTTIMER_SIGNAL );

    // Block timer signal temporarily
    if( block_rttimer() != TIMER_ERROR_NONE )
        return TIMER_ERROR_SIGPROCMASK;

    if( clock_getcpuclockid( controller_pid, &rttimer_clock ) != 0 )
        return TIMER_ERROR_GETCLOCKID;

    // Create the timer
    sevt.sigev_notify = SIGEV_SIGNAL;
    sevt.sigev_signo = RTTIMER_SIGNAL;
    sevt.sigev_value.sival_ptr = &rttimer_id;
    if( timer_create( rttimer_clock, &sevt, &rttimer_id ) == -1 )
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

/// Initializes the controller monitoring timer which determines when the controller has
/// exhausted its budget
void init_rttimer( void ) {

    if( create_rttimer( ) != TIMER_ERROR_NONE ) {
        printf( "Failed to create timer\n" );
        // should probably throw or exit but possible to create a zombie controller as artifact
    }

    if( VERBOSE ) printf("(coordinator) rttimer initialized\n");

    // unblock the controller process
    resume_controller( );
    // Note: this doesn't mean the controller is fully initialized
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
static void* wakeup( void* arg ) {
    while( 1 ) {
        printf( "(wakeup) event\n" );
        char buf = 0;
        write( fd_wakeup_to_coordinator[1], &buf, 1 );
        //usleep( 1 );
    }
    return NULL;
}

//-----------------------------------------------------------------------------

/// Spawns the wakeup thread with a priority level one below the controller thread
/// (therefore two below the coordinator).  For more information see
/// wakeup_coordinator_on_a_blocked_controller( void* ) above
void create_wakeup_thread( void ) {

    int result;
    pthread_attr_t attributes;

    pthread_attr_init( &attributes );

    result = pthread_attr_setinheritsched( &attributes, PTHREAD_EXPLICIT_SCHED );
    if( result != 0 ) {
        printf( "Failed to set explicit schedule attribute for wakeup thread.\n" );
    }

    struct sched_param param;

    result = pthread_attr_setschedpolicy( &attributes, SCHED_RR );
    if( result != 0 ) {
        printf( "Failed to set schedule policy attribute for wakeup thread.\n" );
    }

    param.sched_priority = coordinator_priority - 2;

    result = pthread_attr_setschedparam( &attributes, &param);
    if( result != 0 ) {
        printf( "Failed to set schedule priority attribute for wakeup thread.\n" );
    }

    printf( "creating wakeup\n" );

    pthread_create( &wakeup_thread, &attributes, wakeup, NULL );

    printf( "wakeup created\n" );

    int policy;
    pthread_getschedparam( wakeup_thread, &policy, &param );
    printf( "wakeup priority: %d\n", param.sched_priority );

    pthread_attr_destroy( &attributes );
}

//-----------------------------------------------------------------------------

void init( int argc, char* argv[] ) {

    if( open_error_log( ) != 0 ) {
	exit( 1 );
    }

    coordinator_pid = getpid( );

    // restrict the cpu set s.t. controller only runs on a single processor
    cpu_set_t cpuset_mask;
    // zero out the cpu set
    CPU_ZERO( &cpuset_mask );
    // set the cpu set s.t. controller only runs on 1 processor specified by DEFAULT_PROCESSOR
    CPU_SET( DEFAULT_PROCESSOR, &cpuset_mask );

    if ( sched_setaffinity( 0, sizeof(cpuset_mask), &cpuset_mask ) == -1 ) {
        // there was an error setting the affinity for the coordinator
        // NOTE: can check errno if this occurs
        perror( "sched_setaffinity" );
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
        printf( "There was an error setting the scheduling policy to SCHED_RR for the coordinator.  The actual policy is%s\n",
                (policy == SCHED_OTHER) ? "SCHED_OTHER" :
                (policy == SCHED_FIFO) ? "SCHED_FIFO" :
                "UNDEFINED" );
    }

    // validate the priority
    // INCORRECT RT(99) not 1: generally the expected value is 1 with SCHED_RR, but it may not be the a case depending on platform
    sched_getparam( 0, &param );
    coordinator_priority = param.sched_priority;
    printf( "coordinator process priority: %d\n", coordinator_priority );

    //++++++++++++++++++++++++++++++++++++++++++++++++
/*
    // get processor information
    std::stringstream filename;
    //filename << "/proc/" << (int)coordinator_pid;
    filename << "/proc/cpuinfo";
    FILE* fp = fopen( filename.str().c_str( ), "r" );
    if( fp == 0 ) printf( "error opening processor file\n" );

    while( !feof( fp) ) {
	if( fgets( strbuffer, 256, fp ) == NULL)
	    break;
        fputs( strbuffer, stdout );
    }


    fclose( fp );    
*/
    if( cpuinfo.load( ) != CPUINFO_ERROR_NONE ) {
	printf( "ERROR: Failed to load cpuinfo\n" );
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
	    printf( "ERROR: default processor not found in cpus\n" );
	}
    }
    printf( "cpu speed (MHz): %f\n", cpu_speed_mhz );
    printf( "cpu speed (Hz): %lld\n", cpu_speed_hz );

    //++++++++++++++++++++++++++++++++++++++++++++++++

    // Comm channels must be initialized before any forking or thread creation
    initialize_pipes( );

    amsgbuffer = actuator_msg_buffer_c( ACTUATOR_MSG_BUFFER_NAME, ACTUATOR_MSG_BUFFER_MUTEX_NAME, true );
    amsgbuffer.open( );

    if( VERBOSE ) printf( "coordinator process initialized\n" );

    //++++++++++++++++++++++++++++++++++++++++++++++++
    // initialize dynamics

    init_dynamics( argc, argv );

    //++++++++++++++++++++++++++++++++++++++++++++++++
    // initialize controller

    fork_controller( );

    // ???????????????????????
    suspend_controller( );

    init_rttimer( );

    create_wakeup_thread( );
    // ???????????????????????

    //++++++++++++++++++++++++++++++++++++++++++++++++

    if( open_variance_log( ) != 0 ) {
	exit( 1 );
    }
}

//-----------------------------------------------------------------------------

void shutdown( void ) {
    variance_log.close( );

    pthread_kill( wakeup_thread, SIGKILL );

    shutdown_controller( );
    resume_controller( );

    sleep( 1 );  // block for a second to allow controller to receive signal

    error_log.close( );

    //exit( 0 );
}

//-----------------------------------------------------------------------------
// ENTRY POINT
//-----------------------------------------------------------------------------

#define DYNAMICS_MIN_STEP 0.001

int main( int argc, char* argv[] ) {

    // Due to realtime scheduling, et al, this program must have root access to run.
    if( geteuid() != 0 ) {
        printf( "This program requires root access.  Re-run with sudo.\nExiting.\n" );
        sprintf( strbuffer, "This program requires root access.  Re-run with sudo.\nExiting.\n" );
	error_log.write( strbuffer );
        return 0;
    }

    init( argc, argv );

    char input_buffer;
    //int i = 0;

    int max_fd = std::max( fd_timer_to_coordinator[0], fd_wakeup_to_coordinator[0] );
    max_fd = std::max( max_fd, fd_controller_to_coordinator[0] );

    Real previous_t = 0.0;
    Real current_t = 0.0;
    Real dt;

    unsigned long long ts, ts_start, ts_prev, ts_interval;
    double dts, dts_start, dts_prev, dts_interval;
    rdtscll( ts_start );
    ts_prev = ts_start;

    unblock_rttimer( );

    while( 1 ) {

	if( current_t > 1.0 ) break;

        fd_set fds_channels;

        FD_ZERO( &fds_channels );
        FD_SET( fd_timer_to_coordinator[0], &fds_channels );
        FD_SET( fd_wakeup_to_coordinator[0], &fds_channels );
        FD_SET( fd_controller_to_coordinator[0], &fds_channels );

        // TODO: review the following code
        int result = select( max_fd + 1, &fds_channels, NULL, NULL, NULL );
        if( result ) {
            if( FD_ISSET( fd_timer_to_coordinator[0], &fds_channels ) ) {

                // timer event
                unsigned long long ts_buffer;
                if( read( fd_timer_to_coordinator[0], &ts_buffer, sizeof(unsigned long long) ) == -1 ) {
                    switch( errno ) {
                    case EINTR:
                        // The read operation was terminated due to the receipt of a signal, and no data was transferred.
                        // in this case, there us a read event due to an undetermined reason causing the system to unblock,
                        // but the event is not actually triggered by the controller running for the budget interval.  Instead,
                        // the read event is occurring 'immediately' after the last loop cycle, the duration is effectively zero,
                        // and the controller is not actually running for any significant amount of time.  Trapping this event
                        // prevents a false positive.

                        // However, do not know appriopriate course of action in the architecture
                        break;
                    default:
                        sprintf( strbuffer, "EXCEPTION: unhandled read error in main loop reading from fd_timer_to_coordinator" );
			error_log.write( strbuffer );

                        break;
                    }
                }
		// if this timer event is during the initialization phase of the controller
		// then throw it out
		if( !controller_fully_initialized ) { 
		    ts_prev = ts_buffer;
		    continue;
		}

                suspend_controller( );

		ts = ts_buffer;
		dts = cycles_to_seconds( ts );

		ts_interval = ts - ts_prev;
		dts_interval = cycles_to_seconds( ts_interval );

                //if( VERBOSE ) printf( "(coordinator) Timer Event: %ulld\n", ts );
                //printf( "(coordinator) Timer Event (cycles, seconds): %lld, %f\n", ts, dts );
                printf( "(coordinator) Timer Interval (cycles, seconds): %lld, %f\n", ts_interval, dts_interval );
                //printf( "(coordinator) Timer Previous (cycles, seconds): %lld, %f\n", ts_prev, dts_prev );

		//std::string output;
		//output += ts_interval;
		//variance_log.write( output );
		//variance_log.write( std::to_string( ts_interval ) );

		sprintf( strbuffer, "%lld\n", ts_interval );
		variance_log.write( strbuffer );

		ts_prev = ts;
		dts_prev = cycles_to_seconds( ts_prev );

                resume_controller( );

            } else if( FD_ISSET( fd_controller_to_coordinator[0], &fds_channels ) ) {
                // controller notification event
                if( read( fd_controller_to_coordinator[0], &input_buffer, 1 ) == -1 ) {
                    switch( errno ) {
                    case EINTR:
                        // yet to determine appropriate course of action if this happens
                        break;
                    default:
                        sprintf( strbuffer, "EXCEPTION: unhandled read error in main loop reading from fd_controller_to_coordinator" );
			error_log.write( strbuffer );

                        break;
                    }
                }

                if( VERBOSE ) printf( "(coordinator) received a notification from controller\n" );
                // now check the message buffer and respond to the type of message

                actuator_msg_c msg = amsgbuffer.read();
                if(VERBOSE) msg.print();

                char buf = 0;

                switch( msg.header.type ) {
                case ACTUATOR_MSG_COMMAND:
                    if(VERBOSE) printf( "(coordinator) received ACTUATOR_MSG_COMMAND\n" );

                    suspend_controller( );

                    dynamics_get_command( );

                    resume_controller( );
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

                        msg = amsgbuffer.read( );
                        if(VERBOSE) msg.print();

                        if(VERBOSE) printf( "(coordinator) notifying controller state available\n" );

			// ???????????????????????
			// Should we start timing here rather than in initialization?
			//init_rttimer( );
    			//unblock_rttimer( );
    			//create_wakeup_thread( );
			// ???????????????????????

                        //coordinator_resume_controller( );  // paired with above suspend
                        write( fd_coordinator_to_controller[1], &buf, 1 );
                    } else {
                        // How do we objectively convert system time to sim time?
                        // and vice versa

                        // if dynamics behind controller [
                        // Need an objective query.

                        // right now this time is based on the frequency and cycle of the controller
                        current_t = msg.state.time;

                        dt = current_t - previous_t;
                        printf( "dt: %e\n", dt );

                        run_dynamics( dt );

                        dynamics_state_requested( );

                        msg = amsgbuffer.read( );
                        if(VERBOSE) msg.print();

                        if(VERBOSE) printf( "(coordinator) notifying controller state available\n" );

                        write( fd_coordinator_to_controller[1], &buf, 1 );

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
	        }
            } else if( FD_ISSET( fd_wakeup_to_coordinator[0], &fds_channels ) ) {
                // wakeup event
                if( read( fd_wakeup_to_coordinator[0], &input_buffer, 1 ) == -1 ) {
                    switch( errno ) {
                    case EINTR:
                        // yet to determine appropriate course of action if this happens
                        break;
                    default:
                        sprintf( strbuffer, "EXCEPTION: unhandled read error in main loop reading from fd_wakeup_to_coordinator" );
			error_log.write( strbuffer );

                        break;
                    }
                }

                //printf( "(coordinator) received a wakeup from a blocked controller\n" );
            }
        }
    }
    shutdown( );
}
