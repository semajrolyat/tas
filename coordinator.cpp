/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

coordinator.cpp
-----------------------------------------------------------------------------*/

#include <TAS.h>

//-----------------------------------------------------------------------------

using boost::dynamic_pointer_cast;


//-----------------------------------------------------------------------------
// SHARED RESOURCE MANAGEMENT
//-----------------------------------------------------------------------------

// The shared message buffer
ActuatorMessageBuffer amsgbuffer;

//-----------------------------------------------------------------------------
// PLUGIN MANAGEMENT
//-----------------------------------------------------------------------------

// list of plugins.
// Note: In actuality only support a single plugin at this time.
std::list< DynamicsPlugin > plugins;

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
void coordinator_run_dynamics( Real dt ) {
    if(VERBOSE) printf( "(coordinator) Running Dynamics\n" );

    for( std::list< DynamicsPlugin >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
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
void coordinator_request_dynamics_state( void ) {
    if(VERBOSE) printf( "(coordinator) Requesting Dynamics State\n" );

    for( std::list< DynamicsPlugin >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it->pubstate)( );
    }
}

//-----------------------------------------------------------------------------

void coordinator_instruct_dynamics_to_get_command( void ) {
    if(VERBOSE) printf( "(coordinator) Getting Command\n" );

    for( std::list< DynamicsPlugin >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it->getcommand)( );
    }
}

//-----------------------------------------------------------------------------

void coordinator_init_dynamics( const int& argc, char* argv[] ) {

    DynamicsPlugin plugin;
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
void coordinator_initialize_pipes( void ) {
    int flags;

    // NOTE: a read channel needs to be blocking to force  block waiting for event
    // NOTE: a write channel should be non-blocking (won't ever fill up buffer anyway)

    // Open the timer to coordinator channel for timer events
    if( pipe( fd_timer_to_coordinator ) != 0 )
        throw std::runtime_error( "Failed to open fd_timer_to_coordinator pipe." ) ;
    flags = fcntl( fd_timer_to_coordinator[0], F_GETFL, 0 );
    fcntl( fd_timer_to_coordinator[0], F_SETFL, flags );
    flags = fcntl( fd_timer_to_coordinator[1], F_GETFL, 0 );
    fcntl( fd_timer_to_coordinator[1], F_SETFL, flags | O_NONBLOCK );

    // Open the wakeup to coordinator channel for wakeup (blocking) events
    if( pipe( fd_wakeup_to_coordinator ) != 0 )
        throw std::runtime_error( "Failed to open fd_wakeup_to_coordinator pipe." ) ;
    flags = fcntl( fd_wakeup_to_coordinator[0], F_GETFL, 0 );
    fcntl( fd_wakeup_to_coordinator[0], F_SETFL, flags );
    flags = fcntl( fd_wakeup_to_coordinator[1], F_GETFL, 0 );
    fcntl( fd_wakeup_to_coordinator[1], F_SETFL, flags | O_NONBLOCK );

    // Open the coordinator to controller channel for notifications
    if( pipe( fd_coordinator_to_controller ) != 0 )
        throw std::runtime_error( "Failed to open fd_coordinator_to_controller pipe." ) ;
    flags = fcntl( fd_coordinator_to_controller[0], F_GETFL, 0 );
    fcntl( fd_coordinator_to_controller[0], F_SETFL, flags );
    flags = fcntl( fd_coordinator_to_controller[1], F_GETFL, 0 );
    fcntl( fd_coordinator_to_controller[1], F_SETFL, flags | O_NONBLOCK );

    // Open the controller to coordinator channel for notifications
    if( pipe( fd_controller_to_coordinator ) != 0 )
        throw std::runtime_error( "Failed to open fd_controller_to_coordinator pipe." ) ;
    flags = fcntl( fd_controller_to_coordinator[0], F_GETFL, 0 );
    fcntl( fd_controller_to_coordinator[0], F_SETFL, flags );
    flags = fcntl( fd_controller_to_coordinator[1], F_GETFL, 0 );
    fcntl( fd_controller_to_coordinator[1], F_SETFL, flags | O_NONBLOCK );

}

//-----------------------------------------------------------------------------
// PROCESS MANAGEMENT
//-----------------------------------------------------------------------------

#define DEFAULT_PROCESSOR    0

//-----------------------------------------------------------------------------

// Coordinator process information
static pid_t coordinator_pid;
static int coordinator_priority;

// Controller process information
static int controller_pid;

bool controller_fully_initialized = false;
//-----------------------------------------------------------------------------

/// Creates the controller process with a priority level one step below the coordinator.
void coordinator_fork_controller( void ) {

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
void coordinator_suspend_controller( void ) {

    kill( controller_pid, SIGSTOP );
}

//-----------------------------------------------------------------------------

/// Continue the controller from the coordinator process
void coordinator_resume_controller( void ) {

    kill( controller_pid, SIGCONT );
}

//-----------------------------------------------------------------------------
// TIMER CONTROL
//-----------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------

// Error codes for timer control
#define ERR_TIMER_NOERROR                       0
#define ERR_TIMER_FAILURE_SIGACTION             1
#define ERR_TIMER_FAILURE_SIGPROCMASK           2
#define ERR_TIMER_FAILURE_CREATE                3
#define ERR_TIMER_FAILURE_SETTIME               4
#define ERR_TIMER_FAILURE_GETCLOCKID            5

//-----------------------------------------------------------------------------

// The signal identifier for the controller's real-time timer
#define RTTIMER_SIGNAL                          SIGRTMIN + 4

// timer variables
clockid_t controllers_clock;
sigset_t controllers_rttimer_mask;

// The signal action to reference controller's process signal handler
struct sigaction controllers_rttimer_sigaction;

//-----------------------------------------------------------------------------

/// The signal handler for the controller's timer.  Is called only when the controller's
/// budget is exhausted.  Puts a message out onto the comm channel the coordinator
/// is listening to for notifications about this event
void coordinator_rttimer_controller_budget_monitor_sighandler( int signum, siginfo_t *si, void *data ) {
    printf( "(timer) event\n" );
    char buf = 0;
    write( fd_timer_to_coordinator[1], &buf, 1 );
}

//-----------------------------------------------------------------------------

/// Blocks the timer signal if it is necessary to suppress the timer
int coordinator_block_controllers_signal_rttimer( void ) {
    if( sigprocmask( SIG_SETMASK, &controllers_rttimer_mask, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGPROCMASK;
    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------

/// Unblocks a blocked timer signal
int coordinator_unblock_controllers_signal_rttimer( void ) {
    if( sigprocmask( SIG_UNBLOCK, &controllers_rttimer_mask, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGPROCMASK;
    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------

/// Create the timer to monitor the controller process
int coordinator_create_rttimer_controller_monitor( void ) {
    timer_t timerid;
    struct sigevent sevt;
    struct itimerspec its;

    // Establish handler for timer signal
    controllers_rttimer_sigaction.sa_flags = SA_SIGINFO;
    controllers_rttimer_sigaction.sa_sigaction = coordinator_rttimer_controller_budget_monitor_sighandler;
    sigemptyset( &controllers_rttimer_sigaction.sa_mask );
    if( sigaction( RTTIMER_SIGNAL, &controllers_rttimer_sigaction, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGACTION;

    // intialize the signal mask
    sigemptyset( &controllers_rttimer_mask );
    sigaddset( &controllers_rttimer_mask, RTTIMER_SIGNAL );

    // Block timer signal temporarily
    if( coordinator_block_controllers_signal_rttimer() != ERR_TIMER_NOERROR )
        return ERR_TIMER_FAILURE_SIGPROCMASK;

    if( clock_getcpuclockid( controller_pid, &controllers_clock ) != 0 )
        return ERR_TIMER_FAILURE_GETCLOCKID;

    // Create the timer
    sevt.sigev_notify = SIGEV_SIGNAL;
    sevt.sigev_signo = RTTIMER_SIGNAL;
    sevt.sigev_value.sival_ptr = &timerid;
    if( timer_create( controllers_clock, &sevt, &timerid ) == -1 )
       return ERR_TIMER_FAILURE_CREATE;

    // intial delay for the timer
    its.it_value.tv_sec = DEFAULT_INITDELAY_RTTIMER_SEC;
    its.it_value.tv_nsec = DEFAULT_INITDELAY_RTTIMER_NSEC;
    // budget for the controller
    its.it_interval.tv_sec = DEFAULT_BUDGET_RTTIMER_SEC;
    its.it_interval.tv_nsec = DEFAULT_BUDGET_RTTIMER_NSEC;

    // Set up the timer
    if( timer_settime( timerid, 0, &its, NULL ) == -1 )
        return ERR_TIMER_FAILURE_SETTIME;

    // NOTE: The timer is blocked at this point.  It must be unblocked in
    // the coordinator process for the timer to function

    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------

/// Initializes the controller monitoring timer which determines when the controller has
/// exhausted its budget
void coordinator_init_rttimer_controller_monitor( void ) {

    if( coordinator_create_rttimer_controller_monitor( ) != ERR_TIMER_NOERROR ) {
        printf( "Failed to create timer\n" );
        // should probably throw or exit but possible to create a zombie controller as artifact
    }

    if( VERBOSE ) printf("(coordinator) rttimer initialized\n");

    // unblock the controller process
    coordinator_resume_controller( );
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
static void* wakeup_coordinator_on_a_blocked_controller( void* arg ) {
    while( 1 ) {
        //printf( "(wakeup) event\n" );
        char buf = 0;
        write( fd_wakeup_to_coordinator[1], &buf, 1 );
        usleep( 1 );
    }
}

//-----------------------------------------------------------------------------

/// Spawns the wakeup thread with a priority level one below the controller thread
/// (therefore two below the coordinator).  For more information see
/// wakeup_coordinator_on_a_blocked_controller( void* ) above
void coordinator_create_wakeup_thread( void ) {

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

    pthread_create( &wakeup_thread, &attributes, wakeup_coordinator_on_a_blocked_controller, NULL );

    printf( "wakeup created\n" );

    int policy;
    pthread_getschedparam( wakeup_thread, &policy, &param );
    printf( "wakeup priority: %d\n", param.sched_priority );

    pthread_attr_destroy( &attributes );
}

//-----------------------------------------------------------------------------
// ENTRY POINT
//-----------------------------------------------------------------------------

#define DYNAMICS_MIN_STEP 0.001

int main( int argc, char* argv[] ) {
    // Due to realtime scheduling, et al, this program must have root access to run.
    if( geteuid() != 0 ) {
        printf( "This program requires root access.  Re-run with sudo.\nExiting.\n" );
        return 0;
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
    // generally the expected value is 1 with SCHED_RR, but it may not be the a case depending on platform
    sched_getparam( 0, &param );
    coordinator_priority = param.sched_priority;
    printf( "coordinator process priority: %d\n", coordinator_priority );

    //++++++++++++++++++++++++++++++++++++++++++++++++

    // Comm channels must be initialized before any forking or thread creation
    coordinator_initialize_pipes( );

    amsgbuffer = ActuatorMessageBuffer( ACTUATOR_MESSAGE_BUFFER_NAME, ACTUATOR_MESSAGE_BUFFER_MUTEX_NAME, true );
    amsgbuffer.open( );

    if( VERBOSE ) printf( "coordinator process initialized\n" );

    //++++++++++++++++++++++++++++++++++++++++++++++++

    coordinator_init_dynamics( argc, argv );

    //++++++++++++++++++++++++++++++++++++++++++++++++

    coordinator_fork_controller( );
    coordinator_suspend_controller( );

    coordinator_init_rttimer_controller_monitor( );

    coordinator_create_wakeup_thread( );

    //++++++++++++++++++++++++++++++++++++++++++++++++

    char input_buffer;
    int i = 0;

    int max_fd = std::max( fd_timer_to_coordinator[0], fd_wakeup_to_coordinator[0] );
    max_fd = std::max( max_fd, fd_controller_to_coordinator[0] );

    Real previous_t = 0.0;
    Real current_t = 0.0;
    Real dt;

    while( 1 ) {

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
                if( read( fd_timer_to_coordinator[0], &input_buffer, 1 ) == -1 ) {
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
                        printf( "EXCEPTION: unhandled read error in main loop reading from fd_timer_to_coordinator" );
                        break;
                    }
                }
                coordinator_suspend_controller( );

                if( VERBOSE ) printf( "(coordinator) received a timer event on a controller\n" );

                coordinator_resume_controller( );

            } else if( FD_ISSET( fd_wakeup_to_coordinator[0], &fds_channels ) ) {
                // wakeup event
                if( read( fd_wakeup_to_coordinator[0], &input_buffer, 1 ) == -1 ) {
                    switch( errno ) {
                    case EINTR:
                        // yet to determine appropriate course of action if this happens
                        break;
                    default:
                        printf( "EXCEPTION: unhandled read error in main loop reading from fd_wakeup_to_coordinator" );
                        break;
                    }
                }

                //printf( "(coordinator) received a wakeup from a blocked controller\n" );
            } else if( FD_ISSET( fd_controller_to_coordinator[0], &fds_channels ) ) {
                // controller notification event
                if( read( fd_controller_to_coordinator[0], &input_buffer, 1 ) == -1 ) {
                    switch( errno ) {
                    case EINTR:
                        // yet to determine appropriate course of action if this happens
                        break;
                    default:
                        printf( "EXCEPTION: unhandled read error in main loop reading from fd_controller_to_coordinator" );
                        break;
                    }
                }

                if( VERBOSE ) printf( "(coordinator) received a notification from controller\n" );
                // now check the message buffer and respond to the type of message

                ActuatorMessage msg = amsgbuffer.read();
                if(VERBOSE) msg.print();

                char buf = 0;

                switch( msg.header.type ) {
                case MSG_TYPE_COMMAND:
                    if(VERBOSE) printf( "(coordinator) received MSG_TYPE_COMMAND\n" );

                    coordinator_suspend_controller( );

                    coordinator_instruct_dynamics_to_get_command( );

                    coordinator_resume_controller( );
                    break;
                case MSG_TYPE_STATE_REQUEST:
                    if(VERBOSE) printf( "(coordinator) received MSG_TYPE_STATE_REQUEST\n" );

                    // Note: the controller is blocking now on read from fd_coordinator_to_controller[0]

                    if( !controller_fully_initialized ) {
                        // indicates that the controller is fully initialized and timing can begin
                        // and that this is querying for initial state
                        controller_fully_initialized = true;

                        // therefore suspend the controller and unblock the timer
                        //coordinator_suspend_controller( );    // be careful.  If you suspend must resume
                        //coordinator_unblock_controllers_signal_rttimer( );

                        coordinator_request_dynamics_state( );

                        msg = amsgbuffer.read( );
                        if(VERBOSE) msg.print();

                        if(VERBOSE) printf( "(coordinator) notifying controller state available\n" );

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
                        printf( "dt: %f\n", dt );

                        coordinator_run_dynamics( dt );

                        coordinator_request_dynamics_state( );

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
            }
        }
    }
}
