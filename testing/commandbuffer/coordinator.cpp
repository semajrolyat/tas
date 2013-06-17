/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

Extension of ut_controllerschedule.cpp prototyping.  This test validates
interprocess communication and process manangement over an external controller
program, schedule handling of the controller process and accurate time
slicing of the controller process.
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

#include <sstream>
#include <fstream>
#include <iostream>
#include <list>

#include <stdint.h>

#include <Moby/Simulator.h>
#include <Moby/RCArticulatedBody.h>

#include "CommandBuffer.h"
#include "DynamicsPlugin.h"

//-----------------------------------------------------------------------------

// Verbose
#define VERBOSE 0

//-----------------------------------------------------------------------------

using boost::dynamic_pointer_cast;

//-----------------------------------------------------------------------------

// Sim/Coordinator process information
static int sim_pid;
static int sim_priority;

// Controller process information
static int controller_pid;

// Controller log file
std::ofstream controller_log;

// Time values.  Note: may not be necessary as this file evolves
struct timeval tv_controller_quantum_start_time;
struct timeval tv_controller_quantum_end_time;

struct timeval tv_controller_quantum_interval;

//-----------------------------------------------------------------------------

CommandBuffer cmdbuffer;

//-----------------------------------------------------------------------------
// PLUGIN MANAGEMENT
//-----------------------------------------------------------------------------

// plugin handle
void* HANDLE = NULL;

// list of plugins.
// Note: In actuality only support a single plugin at this time.
std::list< DynamicsPlugin > plugins;

//-----------------------------------------------------------------------------

/**
  given the path to a file (filename), attempts to read dynamics plugin into the set of
  registered plugins.  dynamics plugins must support both init_fun_t and run_fun_t
  interfaces
*/
void read_dynamics_plugin( const char* filename ) {
    // attempt to read the plugin file
    HANDLE = dlopen( filename, RTLD_LAZY );
    if ( !HANDLE ) {
        std::cerr << " failed to read plugin from " << filename << std::endl;
        std::cerr << "  " << dlerror( ) << std::endl;
        exit( -1 );
    }

    DynamicsPlugin plugin;

    // locate the init function
    bool failed = false;
    plugin.init = (init_fun_t) dlsym(HANDLE, "init");
    const char* dlsym_error1 = dlerror( );
    if ( dlsym_error1 ) {
        std::cerr << " warning: cannot load symbol 'init' from " << filename << std::endl;
        std::cerr << "        error follows: " << std::endl << dlsym_error1 << std::endl;
        failed = true;
    }

    // locate the run function
    plugin.run = (run_fun_t) dlsym(HANDLE, "run");
    const char* dlsym_error2 = dlerror( );
    if ( dlsym_error2 ) {
        std::cerr << " warning: cannot load symbol 'run' from " << filename << std::endl;
        std::cerr << "        error follows: " << std::endl << dlsym_error2 << std::endl;
        failed = true;
    }

    // locate the publish state function
    plugin.pubstate = (pub_state_fun_t) dlsym(HANDLE, "publish_state");
    const char* dlsym_error3 = dlerror( );
    if ( dlsym_error3 ) {
        std::cerr << " warning: cannot load symbol 'publish_state' from " << filename << std::endl;
        std::cerr << "        error follows: " << std::endl << dlsym_error3 << std::endl;
        failed = true;
    }

    // if both functions were located then push the pair into the list of plugins
    if( !failed ) plugins.push_back( plugin );
}

//-----------------------------------------------------------------------------

/**
  invokes the intialization function (init_fun_t) of all registered dynamics plugins by
  forwarding command line parameters to the function
*/
void initialize_dynamics( int argc, char** argv ) {
    for( std::list< DynamicsPlugin >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it->init)( argc, argv );
    }
}

//-----------------------------------------------------------------------------

/**
  invokes the run function (run_fun_t) of all the registered dynamics plugins
*/
void run_dynamics( ) {
    if(VERBOSE) printf( "(coordinator) Running Dynamics\n" );

    for( std::list< DynamicsPlugin >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it->run)( );
    }
}

//-----------------------------------------------------------------------------

/**
  a means to publish the state from dynamics is necessary (at least in this architecture)
  because there is a need to seed data to the controller (prime the controller)
  so everything can start.  Once everything is started though, publish state is
  really wrapped into the run_dynamics mechanism
*/
void publish_state( ) {
    if(VERBOSE) printf( "(coordinator) Requesting Dynamics State\n" );

    for( std::list< DynamicsPlugin >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it->pubstate)( );
    }
}

//-----------------------------------------------------------------------------
// INTERPROCESS COMMUNICATION
//-----------------------------------------------------------------------------

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
struct sigaction sigaction_controller;

//-----------------------------------------------------------------------------

// The actual signal handler for the controller timer
void sighandler_timer_controller( int signum, siginfo_t *si, void *data ) {
    char buf;
    write( fd_timercontroller_to_coordinator[1], &buf, 1 );
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

// The signal identifier for the controller's timer
#define SIG_TIMER_CONTROLLER                    SIGRTMIN + 4

// Controller timer variables
clockid_t controller_clock;
sigset_t controller_timer_mask;

//-----------------------------------------------------------------------------
// Blocks the controller timer signal if it is necessary to suppress the timer
// Note: does not affect the actual controller process, just the signal so
// Probably only necessary to use during intialization when process may
// not be initialized but timer has been
int block_controller_timer( ) {
    if( sigprocmask( SIG_SETMASK, &controller_timer_mask, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGPROCMASK;
    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------
// Unblocks a blocked controller timer
int unblock_controller_timer( ) {
    if( sigprocmask( SIG_UNBLOCK, &controller_timer_mask, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGPROCMASK;
    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------
// Create the timer for the controller process
int create_controller_timer( ) {
    timer_t timerid;
    struct sigevent sevt;
    struct itimerspec its;

    // Establish handler for timer signal
    sigaction_controller.sa_flags = SA_SIGINFO;
    sigaction_controller.sa_sigaction = sighandler_timer_controller;
    sigemptyset( &sigaction_controller.sa_mask );
    if( sigaction( SIG_TIMER_CONTROLLER, &sigaction_controller, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGACTION;

    // intialize the signal mask
    sigemptyset( &controller_timer_mask );
    sigaddset( &controller_timer_mask, SIG_TIMER_CONTROLLER );

    // Block timer signal temporarily
    if( block_controller_timer() != ERR_TIMER_NOERROR )
        return ERR_TIMER_FAILURE_SIGPROCMASK;

    if( clock_getcpuclockid( controller_pid, &controller_clock ) != 0 )
        return ERR_TIMER_FAILURE_GETCLOCKID;

    // Create the timer
    sevt.sigev_notify = SIGEV_SIGNAL;
    sevt.sigev_signo = SIG_TIMER_CONTROLLER;
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
#define DEFAULT_CONTROLLER_PROGRAM      "commandbuffer_controller"
#define DEFAULT_CONTROLLER_PROCESSOR    0

//-----------------------------------------------------------------------------
// Forks off a the controller process.  Executes the external controller
// program
void fork_controller( ) {

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
        // set the cpu set s.t. controller only runs on 1 processor specified by DEFAULT_CONTROLLER_PROCESSOR
        CPU_SET( DEFAULT_CONTROLLER_PROCESSOR, &cpuset_mask );
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
void suspend_controller( ) {

    kill( controller_pid, SIGSTOP );

    if(VERBOSE) printf( "(coordinator) Suspended Controller\n" );
}

//-----------------------------------------------------------------------------
// Unpause the controller process
void resume_controller( ) {

    if(VERBOSE) printf( "(coordinator) Resuming Controller\n" );

    kill( controller_pid, SIGCONT );
}

//-----------------------------------------------------------------------------
// Simulator Entry Point
//-----------------------------------------------------------------------------

//extern boost::shared_ptr<Moby::Simulator> sim;

int main( int argc, char* argv[] ) {

    /// 1. load dynamics references (plugins)
    read_dynamics_plugin( "/home/james/tas/build/commandbuffer/libcommandbuffer_dynamics.so" );

    /// 2. initialize dynamics references
    initialize_dynamics( argc, argv );

    /// 3. initialize controllers
    controller_pid = 0;

    sim_pid = getpid( );
    // hopefully this is highest priority value, i.e. 0.
    // in testing, this has been true, but it's possible the OS will assign a lower priority
    // which may not have significant impact on the coordinator but may affect controller
    sim_priority = getpriority( PRIO_PROCESS, sim_pid );

    // fork the controller then immediately block it
    // NOTE: the controller process may need time to run through initialization
    // which may be delayed by blocking s.t. the initial quantum may get out of
    // step as a result.  I don't have a solution to this potential problem at the moment.
    // FOLLOW UP: Wait to start timing controller on initial perception request
    fork_controller( );
    suspend_controller( );

    // controller has no need to know about these channels
    // comm done completely in the parent process hence init after fork
    initialize_timer_pipes( );

    if( create_controller_timer( ) != ERR_TIMER_NOERROR ) {
        printf( "Failed to create controller timer\n" );
        // should probably throw or exit but possible to create a zombie controller as artifact
    }

    // unblock the controller timer and unblock the controller process
    unblock_controller_timer( );
    resume_controller( );

    //********** Unique to this architecture **********

    // Create a commandbuffer to listen to the commands.  With no timing, in this architecture
    // need to snoop on the issuing of commands so the coordinator knows when to switch between
    // processes.
    cmdbuffer = CommandBuffer( sim_pid );
    cmdbuffer.open();   // TODO : sanity/safety checking

    // have to publish the initial state so that the controller has data to work with
    // otherwise in this particular architecture the controller is blocking on read and
    // coordinator is blocking on read so nothing happens.
    publish_state( );

    /// 4. Start main loop
    while( 1 ) {
        // should block here until a command issued.  Reading in coordinator is only
        // necessary to ensure timing in this architecture
        Command cmd = cmdbuffer.read();
        if(VERBOSE) cmd.print();

        // without listening to commands then the problem here is the controller
        // may not have completed a computation by the time the suspend is issued
        // and in fact may never have time to issue a command due to rapid blocking
        suspend_controller( );

        // now that a command has been issued and the controller is suspended
        // can run dynamics.
        // Note: dynamics needs to accept a Real time dt to run for.
        run_dynamics( );

        // now that dynamics has run for its time slice, resume the controller and
        // go back to the beginning of the coordinator loop
        resume_controller( );
    }

    if( HANDLE != NULL )
        dlclose( HANDLE );

    return 0;
}
