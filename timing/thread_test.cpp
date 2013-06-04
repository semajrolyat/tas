/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

coordinator.cpp
-----------------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <signal.h>
#include <stdexcept>
#include <sys/time.h>
#include <sys/resource.h>
#include <errno.h>
#include <sched.h>
#include <dlfcn.h>
#include <sys/types.h>

#include <sstream>
#include <fstream>
#include <iostream>
#include <list>

#include <stdint.h>

#include <pthread.h>

#include <Moby/Simulator.h>
#include <Moby/RCArticulatedBody.h>

#include "ActuatorMessage.h"
#include "DynamicsPlugin.h"

//#include "monitor.cpp"

//-----------------------------------------------------------------------------

// Verbose
#define VERBOSE 1

//-----------------------------------------------------------------------------

using boost::dynamic_pointer_cast;

//-----------------------------------------------------------------------------

// Sim/Coordinator process information
static pid_t sim_pid;
static int sim_priority;

//static int monitor_tid;

// Controller process information
static int controller_pid;

// Controller log file
std::ofstream controller_log;

// Time values.  Note: may not be necessary as this file evolves
struct timeval tv_controller_quantum_start_time;
struct timeval tv_controller_quantum_end_time;

struct timeval tv_controller_quantum_interval;

//-----------------------------------------------------------------------------

//ActuatorMessageBuffer amsgbuffer_monitor;
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
void initialize_dynamics( int argc, char** argv ) {
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
void publish_state( void ) {
    if(VERBOSE) printf( "(coordinator) Requesting Dynamics State\n" );

    for( std::list< DynamicsPlugin >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it->pubstate)( );
    }
}


void get_command( void ) {
    if(VERBOSE) printf( "(coordinator) Getting Command\n" );

    for( std::list< DynamicsPlugin >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it->getcommand)( );
    }
}

//-----------------------------------------------------------------------------
// INTERPROCESS COMMUNICATION
//-----------------------------------------------------------------------------
/*
// Pipe between controller timer signal handler and coordinator process
int fd_timercontroller_to_coordinator[2];

//-----------------------------------------------------------------------------

// Initializes the Pipes used to communicate between signal handlers and
// coordinator process
void initialize_timer_pipes( void ) {
    int flags;

    // Open the controller timer to coordinator channel
    if( pipe( fd_timercontroller_to_coordinator ) != 0 )
        throw std::runtime_error( "Failed to open timercontroller_to_coordinator pipe." ) ;

    // NOTE: read channel needs to be blocking to cause coordinator to block waiting for
    // timer expiration message
    flags = fcntl( fd_timercontroller_to_coordinator[0], F_GETFL, 0 );
    fcntl( fd_timercontroller_to_coordinator[0], F_SETFL, flags );

    // NOTE: write channel should be non-blocking
    flags = fcntl( fd_timercontroller_to_coordinator[1], F_GETFL, 0 );
    fcntl( fd_timercontroller_to_coordinator[1], F_SETFL, flags | O_NONBLOCK );

}
*/
//-----------------------------------------------------------------------------
// SIGNAL HANDLING
//-----------------------------------------------------------------------------

// Nanoseconds/Second
#define NSECS_PER_SEC 1E9
#define USECS_PER_SEC 1E6

// Quantum for Controller Timer
//#define DEFAULT_NSEC_QUANTUM_CONTROLLER         3E7         // 30 milliseconds
//#define DEFAULT_SEC_QUANTUM_CONTROLLER          1           // 1 second
#define DEFAULT_QUANTUM_CONTROLLER_NSEC         1E8         // 0.1 second
//#define DEFAULT_QUANTUM_CONTROLLER_NSEC         1E7         // 0.01 second
#define DEFAULT_QUANTUM_CONTROLLER_SEC          0           // 0 second

// Initial Delay for Controller Timer
// NOTE: There has to be some delay otherwise timer is disarmed
#define DEFAULT_INITDELAY_CONTROLLER_NSEC       1           // 1 nanoseconds
#define DEFAULT_INITDELAY_CONTROLLER_SEC        0           // 0 second

//-----------------------------------------------------------------------------

// The signal action to reference controller's process signal handler
struct sigaction controller_rttimer_sigaction;

//-----------------------------------------------------------------------------

// The actual signal handler for the controller timer
void controller_rttimer_sighandler( int signum, siginfo_t *si, void *data ) {
    char buf;
    //write( fd_timercontroller_to_coordinator[1], &buf, 1 );

    // write the actuator state out to the command buffer
    ActuatorMessage msg = ActuatorMessage( );
    msg.header.type = MSG_TYPE_NO_COMMAND;
    //msg.state.time = sim->current_time;
    printf( "(dynamics::publish_state)" );
    //msg.print();
    //amsgbuffer_monitor.write( msg );
    //amsgbuffer_monitor.swap( );
}

//-----------------------------------------------------------------------------
// TIMER CONTROL
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
#define CONTROLLER_RTTIMER_SIGNAL                    SIGRTMIN + 4

// timer variables
clockid_t controller_clock;
sigset_t controller_rttimer_mask;

//-----------------------------------------------------------------------------
// Blocks the timer signal if it is necessary to suppress the timer
// Note: does not affect the actual controller process, just the signal so
// Probably only necessary to use during intialization when process may
// not be initialized but timer has been
int controller_block_signal_rttimer( void ) {
    if( sigprocmask( SIG_SETMASK, &controller_rttimer_mask, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGPROCMASK;
    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------
// Unblocks a blocked timer signal
int controller_unblock_signal_rttimer( void ) {
    if( sigprocmask( SIG_UNBLOCK, &controller_rttimer_mask, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGPROCMASK;
    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------
// Create the timer to monitor the controller process
int controller_create_rttimer( void ) {
    timer_t timerid;
    struct sigevent sevt;
    struct itimerspec its;

    // Establish handler for timer signal
    controller_rttimer_sigaction.sa_flags = SA_SIGINFO;
    controller_rttimer_sigaction.sa_sigaction = controller_rttimer_sighandler;
    sigemptyset( &controller_rttimer_sigaction.sa_mask );
    if( sigaction( CONTROLLER_RTTIMER_SIGNAL, &controller_rttimer_sigaction, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGACTION;

    // intialize the signal mask
    sigemptyset( &controller_rttimer_mask );
    sigaddset( &controller_rttimer_mask, CONTROLLER_RTTIMER_SIGNAL );

    // Block timer signal temporarily
    if( controller_block_signal_rttimer() != ERR_TIMER_NOERROR )
        return ERR_TIMER_FAILURE_SIGPROCMASK;

    if( clock_getcpuclockid( controller_pid, &controller_clock ) != 0 )
        return ERR_TIMER_FAILURE_GETCLOCKID;

    // Create the timer
    sevt.sigev_notify = SIGEV_SIGNAL;
    sevt.sigev_signo = CONTROLLER_RTTIMER_SIGNAL;
    sevt.sigev_value.sival_ptr = &timerid;
    if( timer_create( controller_clock, &sevt, &timerid ) == -1 )
       return ERR_TIMER_FAILURE_CREATE;

    // intial delay for the timer
    its.it_value.tv_sec = DEFAULT_INITDELAY_CONTROLLER_SEC;
    its.it_value.tv_nsec = DEFAULT_INITDELAY_CONTROLLER_NSEC;
    // quantum interval for the timer
    // NOTE: technically in this implementation this is a tick not the quantum
    its.it_interval.tv_sec = DEFAULT_QUANTUM_CONTROLLER_SEC;
    its.it_interval.tv_nsec = DEFAULT_QUANTUM_CONTROLLER_NSEC;

    // Set up the timer
    if( timer_settime( timerid, 0, &its, NULL ) == -1 )
        return ERR_TIMER_FAILURE_SETTIME;

    // NOTE: The timer is blocked at this point.  It must be unblocked in
    // the parent process for the timer to function

    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------
// PROCESS MANAGEMENT
//-----------------------------------------------------------------------------

// controller defaults
#define DEFAULT_CONTROLLER_PROGRAM      "timing_controller"
#define DEFAULT_PROCESSOR    0

//-----------------------------------------------------------------------------
// Forks off a the controller process.  Executes the external controller
// program
void fork_controller( void ) {

    controller_pid = fork( );
    if ( controller_pid < 0 ) {
        throw std::runtime_error( "Failed to fork controller." ) ;
    }
    if ( controller_pid == 0 ) {
        controller_pid = getpid( );

        // restrict the cpu set s.t. controller only runs on a single processor
        cpu_set_t cpuset_mask;
        // zero out the cpu set
        CPU_ZERO( &cpuset_mask );
        // set the cpu set s.t. controller only runs on 1 processor specified by DEFAULT_PROCESSOR
        CPU_SET( DEFAULT_PROCESSOR, &cpuset_mask );
        if ( sched_setaffinity( controller_pid, sizeof(cpuset_mask), &cpuset_mask ) == -1 ) {
            // there was an error setting the affinity for the controller
            // NOTE: can check errno if this occurs
            printf( "ERROR: Failed to set affinity for controller process.\n" );
            // could throw or exit or perror
        }

        int sched_policy = sched_getscheduler( controller_pid );
        if ( sched_policy == -1 ) {
            // there was an error getting the scheduler policy for the controller
            // NOTE: can check errno if this occurs
            printf( "ERROR: Failed to get scheduler policy for controller process.\n" );
            // could throw or exit or perror
        } else {
            struct sched_param params;
            params.sched_priority = sim_priority + 1;
            // set the policy
            // NOTE: Round Robin is opted for in this case.  FIFO an option.
            // man pages claim that either of these policies are real-time
            sched_setscheduler( controller_pid, SCHED_RR, &params );
        }

        /*
        // testing sanity check ... TO BE COMMENTED
        int ret = sched_getaffinity( controller_pid, sizeof(cpuset_mask), &cpuset_mask );
        printf( " sched_getaffinity = %d, cpuset_mask = %08lx\n", sizeof(cpuset_mask), cpuset_mask );
        */

        // execute the controller program
        execl( DEFAULT_CONTROLLER_PROGRAM, "controller", 0 );
        /// Question: How can the controller's robot reference even be set?

        // exit on fail to exec
        _exit( 1 );
    }
}

//-----------------------------------------------------------------------------
// Block/wait the controller process
// Note: Timer signal will be blocked if the controller is blocked due to
// construction of the interface of the signal handler.  Therefore, using
// this method effectively pauses the timer as well so this function is
// a catch all for managing signal suspension and process suspension
void suspend_controller( void ) {

    kill( controller_pid, SIGSTOP );

    if(VERBOSE) printf( "(coordinator) Suspended Controller\n" );
}

//-----------------------------------------------------------------------------
// Unpause the controller process
void resume_controller( void ) {

    if(VERBOSE) printf( "(coordinator) Resuming Controller\n" );

    kill( controller_pid, SIGCONT );
}


//-----------------------------------------------------------------------------
/// Initialize the scheduling policy, priority and affinity for the coordinator
/// process
void coordinator_init( void ) {
    sim_pid = getpid( );

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
    param.sched_priority = sched_get_priority_min( SCHED_RR );
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
    sim_priority = param.sched_priority;
    //printf( "sim_priority: %d\n", sim_priority );

    if( VERBOSE ) printf( "coordinator initialized\n" );
}
//-----------------------------------------------------------------------------

#define DYNAMICS_PLUGIN "/home/james/tas/build/timing/libtiming_dynamics.so"

void dynamics_init( const int& argc, char* argv[] ) {

    DynamicsPlugin plugin;
    int result = plugin.read( DYNAMICS_PLUGIN );	// TODO: Sanity/Safety checking
    plugin.init( argc, argv );
    //initialize_dynamics( argc, argv );

    // if both functions were located then push the pair into the list of plugins
    if( !result ) plugins.push_back( plugin )
;
    if( VERBOSE ) printf("(coordinator) dynamics initialized\n");
}

//-----------------------------------------------------------------------------

void controller_init( void ) {

    controller_pid = 0;

    // fork the controller then immediately block it
    // NOTE: the controller process may need time to run through initialization
    // which may be delayed by blocking s.t. the initial quantum may get out of
    // step as a result.  I don't have a solution to this potential problem at the moment.
    // FOLLOW UP: Wait to start timing controller on initial perception request
    fork_controller( );
    suspend_controller( );

    if( VERBOSE ) printf("(coordinator) controller initialized\n");
}

//-----------------------------------------------------------------------------

void controller_rttimer_init( void ) {

    // may need thread creation here
    //monitor_tid = gettid( );

    // unblock the controller timer and unblock the controller process
    controller_unblock_signal_rttimer( );
    resume_controller( );

    if( controller_create_rttimer( ) != ERR_TIMER_NOERROR ) {
        printf( "Failed to create controller timer\n" );
        // should probably throw or exit but possible to create a zombie controller as artifact
    }

    //amsgbuffer_monitor = ActuatorMessageBuffer( 8811, sim_pid, monitor_tid );
    //amsgbuffer_monitor.open( );   // TODO : sanity/safety checking

    if( VERBOSE ) printf("(coordinator) controller rttimer initialized\n");
}


//-----------------------------------------------------------------------------

static void* wakeup_coordinator( void* arg ) {
    while( 1 ) {
        // write to pipe
        // sleep?  Think that must block somehow!  Write should transfer control back to coordinator though.
    }
}

//-----------------------------------------------------------------------------
//struct thread_info *tinfo;
pthread_t* pt;

void create_wakeup_thread( void ) {
    pthread_create( pt, NULL, wakeup_coordinator, NULL );
}

//-----------------------------------------------------------------------------

void clear_buffer( void ) {
  // sleep before flush to give everybody time to read the message
  usleep(10);
  // Note: sleep arbitrary time - probably some factor of the number of controllers.
  amsgbuffer.flush( );
  amsgbuffer.swap( );
}

//-----------------------------------------------------------------------------
// Simulation Entry Point
//-----------------------------------------------------------------------------

#define DYNAMICS_MIN_STEP 0.001

int main( int argc, char* argv[] ) {

    // Due to realtime scheduling, et al, this program must have root access to run.
    if( geteuid() != 0 ) {
        printf( "This program requires root access.  Re-run with sudo.\nExiting.\n" );
        return 0;
    }

    coordinator_init();

    // Create a commandbuffer to listen to the commands.  With no timing, in this architecture
    // need to snoop on the issuing of commands so the coordinator knows when to switch between
    // processes.
    amsgbuffer = ActuatorMessageBuffer( 8811, sim_pid, 0 );
    amsgbuffer.open();   // TODO : sanity/safety checking

    /*
    dynamics_init( argc, argv );
    controller_init( );
    controller_rttimer_init( );

    while( 1 ) {
        // should block here until a command issued.  Reading in coordinator is only
        // necessary to ensure timing in this architecture
        ActuatorMessage msg = amsgbuffer.read();
        if(VERBOSE) msg.print();

        switch( msg.header.type ) {
        case MSG_TYPE_NO_COMMAND:
        if(VERBOSE) printf( "(coordinator) receied MSG_TYPE_NO_COMMAND\n" );
            suspend_controller( );
            run_dynamics( DYNAMICS_MIN_STEP );
            resume_controller( );
            break;
        case MSG_TYPE_COMMAND:
        if(VERBOSE) printf( "(coordinator) receied MSG_TYPE_COMMAND\n" );
            suspend_controller( );
        clear_buffer( );
            get_command( );
        //Real dt = DYNAMICS_MIN_STEP;
            // Note: may need to store previous msg/time to compute diff for dt
            //dt = msg.state.time;
            run_dynamics( DYNAMICS_MIN_STEP );
            resume_controller( );
            break;
        case MSG_TYPE_STATE_REQUEST:
        if(VERBOSE) printf( "(coordinator) receied MSG_TYPE_STATE_REQUEST\n" );
            suspend_controller( );
        clear_buffer( );
            publish_state( );
            resume_controller( );
            //usleep(1);
            break;
        case MSG_TYPE_STATE_REPLY:
        if(VERBOSE) printf( "(coordinator) receied MSG_TYPE_STATE_REPLY\n" );
            // should be received directly by controller
            resume_controller( );
        default:
            //run_dynamics( 0.001 );
            //run_dynamics( 0.1 );
            // discard
            break;
        }
    clear_buffer( );
    }
    */
    return 0;
}
