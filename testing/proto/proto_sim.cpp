/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

This is an early prototype of the simulation (coordination) process.  Mostly
used to prove process management and interprocess communication.
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

#include <fstream>

//-----------------------------------------------------------------------------
// REFACTORING NOTES
//-----------------------------------------------------------------------------
/*
NOTE: Currently has exception handlers that print and possible kill the process.
Bad juju.  Should refactor to avoid.  Also need to refactor out all printf's
as the buffering is not compatible with asynchronicity and spawned processes
don't have access to stdio in the parent process.
*/

//-----------------------------------------------------------------------------

#define _DEBUG_TO_STDOUT 1

//-----------------------------------------------------------------------------

static int sim_pid;
static int sim_priority;

static int controller_pid;
static int sensor_pid;

//-----------------------------------------------------------------------------

// Pipe to communicate messages from controller to coordinator
int fd_controller_to_coordinator[2];
// Pipe to communicate messages from coordinator to controller
int fd_coordinator_to_controller[2];
// Pipe to communicate messages from sensor to coordinator
int fd_sensor_to_coordinator[2];
// What about pipe from coordinator to sensor??


// Pipes to communicate messages from timers to coordinator
int fd_timercontroller_to_coordinator[2];


//-----------------------------------------------------------------------------

/**
  Initializes all of the interprocess communication pipes in the Top Level Process
  Forked processes inherit open file descriptors and so can use the pipes
  to read/write messages.  However, forked processes are responsible for closing
  pipes (a must close pipes) they shouldn't use so that there are no problems
  with a child process cross talking on a channel that they shouldn't message on
*/
// NOTE: Should not be using throw in the long run.  Ok for prototype, but
// should use return type as int to indicate/enumerate failure
void initialize_ipc_pipes( void ) {
    int flags;

    // Open the controller to coordinator channel
    if( pipe( fd_controller_to_coordinator ) != 0 )
        throw std::runtime_error( "Failed to open controller_to_coordinator pipe." ) ;
    flags = fcntl( fd_controller_to_coordinator[0], F_GETFL, 0 );
    fcntl( fd_controller_to_coordinator[0], F_SETFL, flags | O_NONBLOCK );
    flags = fcntl( fd_controller_to_coordinator[1], F_GETFL, 0 );
    fcntl( fd_controller_to_coordinator[1], F_SETFL, flags | O_NONBLOCK );

    // Open the coordinator to controller channel
    if( pipe( fd_coordinator_to_controller ) != 0 )
        throw std::runtime_error( "Failed to open coordinator_to_controller pipe." ) ;
    flags = fcntl( fd_coordinator_to_controller[0], F_GETFL, 0 );
    fcntl( fd_coordinator_to_controller[0], F_SETFL, flags | O_NONBLOCK );
    flags = fcntl( fd_coordinator_to_controller[1], F_GETFL, 0 );
    fcntl( fd_coordinator_to_controller[1], F_SETFL, flags | O_NONBLOCK );

    // Open the sensor to coordinator channel
    if( pipe( fd_sensor_to_coordinator ) != 0 )
        throw std::runtime_error( "Failed to open sensor_to_dynamics pipe." ) ;
    flags = fcntl( fd_sensor_to_coordinator[0], F_GETFL, 0 );
    fcntl( fd_sensor_to_coordinator[0], F_SETFL, flags | O_NONBLOCK );
    flags = fcntl( fd_sensor_to_coordinator[1], F_GETFL, 0 );
    fcntl( fd_sensor_to_coordinator[1], F_SETFL, flags | O_NONBLOCK );

    // If coodinator to sensor channel required, add opening it here
}

//-----------------------------------------------------------------------------
/**
    Initializes the internal communication channels for the coordinator to
    receive messages from the timer signal handlers.  Should not be invoked
    before the processes are forked as the subprocesses SHALL NOT communicate
    on these channels.  Exclusive internal channels for the coordinator
*/
// NOTE: Should not be using throw in the long run.  Ok for prototype, but
// should use return type as int to indicate/enumerate failure
void initialize_timer_pipes( void ) {
    int flags;

    // Open the controller timer to coordinator channel
    if( pipe( fd_timercontroller_to_coordinator ) != 0 )
        throw std::runtime_error( "Failed to open timercontroller_to_coordinator pipe." ) ;

    // Note: read channel needs to be blocking to cause coordinator to block waiting for
    // timer expiration
    flags = fcntl( fd_timercontroller_to_coordinator[0], F_GETFL, 0 );
    //fcntl( fd_timercontroller_to_coordinator[0], F_SETFL, flags | O_NONBLOCK );
    fcntl( fd_timercontroller_to_coordinator[0], F_SETFL, flags );

    flags = fcntl( fd_timercontroller_to_coordinator[1], F_GETFL, 0 );
    //fcntl( fd_timercontroller_to_coordinator[1], F_SETFL, flags | O_NONBLOCK );
    fcntl( fd_timercontroller_to_coordinator[1], F_SETFL, flags | O_NONBLOCK );

}
//-----------------------------------------------------------------------------

/**
  Sensor process
*/
void sensor( void ) {
    char buf;

    // 0 read channel, 1 write channel
    // sensor uses only the write to coordinator channel on this pipe
    close( fd_sensor_to_coordinator[0] );
    // N.B. If there is a need to open 2 way comm then don't close this pipe.
    // May be necessary to send dynamics state to sensor.

    // sensor doesnt use any of these pipes
    close( fd_controller_to_coordinator[0] );
    close( fd_controller_to_coordinator[1] );
    close( fd_coordinator_to_controller[0] );
    close( fd_coordinator_to_controller[1] );

    while(1) {
        usleep(33333);  // 33333 microseconds -> 0.033333 secs -> 1/30 sec/frame
        buf = 1;        // some data to write for testing.  Future this will be sensor state
        write( fd_sensor_to_coordinator[1], &buf, 1 );
    }

    // cleanup - unreachable but noted for now
    close( fd_sensor_to_coordinator[1] );
}

//-----------------------------------------------------------------------------

std::ofstream controller_log;

/**
  Controller Process.  Sends Control messages to
  */

void controller( void ) {
    char buf;
    bool command_generated = false;
    bool request_state = false;

    // 0 read channel, 1 write channel
    // controller uses only the write to coordinator channel on this pipe
    close( fd_controller_to_coordinator[0] );
    // controller uses only the read from coordinator channel on this pipe
    close( fd_coordinator_to_controller[1] );

    // controller doesnt use any of these pipes
    close( fd_sensor_to_coordinator[0] );
    close( fd_sensor_to_coordinator[1] );

    while(1) {
        // attempt to read a single byte from the coordinator channel
        // if there is data to read then there is data to process in the shared memory area

        /*
        while (read(fd_coordinator_to_controller[0], &buf, 1) > 0) {
            // Receiving State info from controller

//            write(fd_controller_to_coordinator[1], &buf, 1);
//            if( _DEBUG_TO_STDOUT ) {
//                printf( "Controlling\n" );
//            }

            if( _DEBUG_TO_STDOUT ) {
                printf( "Controller Received State Message\n" );
            }
        }
        if( request_state ) {
            write( fd_controller_to_coordinator[1], &buf, 1 );
            if( _DEBUG_TO_STDOUT ) {
                printf( "Controller Sent Request State Message\n" );
            }
        }
        if( command_generated ) {
            write( fd_controller_to_coordinator[1], &buf, 1 );
            if( _DEBUG_TO_STDOUT ) {
                printf( "Controller Sent Command Message\n" );
            }
        }
        */

        /*
        int val;
        for( int i = 1; i < 10000; i++ ) {
            val *= i;
        }
        */
        struct timeval tv_timenow;

        gettimeofday( &tv_timenow, NULL );

        //clock_gettime( controller_clock, &ts );

        controller_log << "Time of Day: " << tv_timenow.tv_sec << "\n";

    }

    // cleanup - unreachable but noted for now
    close( fd_controller_to_coordinator[1] );
    close( fd_coordinator_to_controller[0] );

    controller_log.close();
}

//-----------------------------------------------------------------------------
// SIGNAL HANDLING
//-----------------------------------------------------------------------------
// Any signal handlers should be implemented here.  Any state control
// required in the controller (main) loop commanded by signals needs to be piped to
// the controller (per Gabe's warning below)
//
// Note from Gabe:  Signal Handlers should never be within main loop b/c can't do
// anything complex, e.g. can't take a lock
// Signal Handler should write to a pipe that is read by the main loop

#define NSECS_PER_SEC 1E9
#define DEFAULT_NSEC_QUANTUM_CONTROLLER 3E7                 // 30 milliseconds
//#define DEFAULT_SEC_QUANTUM_CONTROLLER 1                 // 1 second
//#define DEFAULT_NSEC_QUANTUM_CONTROLLER 1E8                 // 0.1 second
#define DEFAULT_NSEC_QUANTUM_DYNAMICS 1E7                   // 10 milliseconds

struct sigaction sigaction_contoller;
//struct sigaction sigaction_dynamics;

void sighandler_timer_controller( int signum, siginfo_t *si, void *data ) {
    char buf;
    write( fd_timercontroller_to_coordinator[1], &buf, 1 );
}

//-----------------------------------------------------------------------------
// TIMER CONTROL
//-----------------------------------------------------------------------------

#define ERR_TIMER_NOERROR                       0
#define ERR_TIMER_FAILURE_SIGACTION             1
#define ERR_TIMER_FAILURE_SIGPROCMASK           2
#define ERR_TIMER_FAILURE_CREATE                3
#define ERR_TIMER_FAILURE_SETTIME               4
#define ERR_TIMER_FAILURE_GETCLOCKID            5

/*
Unlike standard signals, real-time signals have no predefined meanings:
the entire set of real-time signals can be used for application-defined purposes.
(Note, however, that the LinuxThreads implementation uses the first three
real-time signals.)
Therefore, SIGRTMIN+n must be greater than (or equal to) 3
*/

#define SIG_TIMER_CONTROLLER                    SIGRTMIN + 4

clockid_t controller_clock;

//-----------------------------------------------------------------------------
/**
    Creates the controller quantum timer
*/
int create_timer_controller() {

    timer_t timerid;
    struct sigevent sevt;
    struct itimerspec its;
    sigset_t mask;

    // Establish handler for timer signal
    sigaction_contoller.sa_flags = SA_SIGINFO;
    sigaction_contoller.sa_sigaction = sighandler_timer_controller;
    sigemptyset( &sigaction_contoller.sa_mask );
    if( sigaction( SIG_TIMER_CONTROLLER, &sigaction_contoller, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGACTION;

    // Block timer signal temporarily
    sigemptyset( &mask );
    sigaddset( &mask, SIG_TIMER_CONTROLLER );
    if( sigprocmask( SIG_SETMASK, &mask, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGPROCMASK;

    if( clock_getcpuclockid( controller_pid, &controller_clock ) != 0 )
        return ERR_TIMER_FAILURE_GETCLOCKID;

    // Create the timer
    sevt.sigev_notify = SIGEV_SIGNAL;
    sevt.sigev_signo = SIG_TIMER_CONTROLLER;
    sevt.sigev_value.sival_ptr = &timerid;
    //if( timer_create( CLOCK_REALTIME, &sevt, &timerid ) == -1 )
    if( timer_create( controller_clock, &sevt, &timerid ) == -1 )
       return ERR_TIMER_FAILURE_CREATE;

    // Start the timer
    its.it_value.tv_sec = 1;                                   // initial delay
    its.it_value.tv_nsec = 0;
    //its.it_interval.tv_sec = DEFAULT_SEC_QUANTUM_CONTROLLER;
    //its.it_interval.tv_nsec = 0;

    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = DEFAULT_NSEC_QUANTUM_CONTROLLER;  // regular quantum

    if( timer_settime( timerid, 0, &its, NULL ) == -1 )
        return ERR_TIMER_FAILURE_SETTIME;

    // Unblock the timer
    if( sigprocmask( SIG_UNBLOCK, &mask, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGPROCMASK;

    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------
/**
    Suspends the controller quantum timer
*/
int suspend_timer_controller() {
    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------
/**
    Resumes the controller quantum timer
*/
int resume_timer_controller() {
    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------
// Process Control
//-----------------------------------------------------------------------------
/**
    Forks off the controller process at a lower priority than the coordinator
    process
*/
void fork_controller() {
    // Fork the Controller
    controller_pid = fork();
    if (controller_pid < 0) {
        throw std::runtime_error( "Failed to fork controller." ) ;
    }
    if (controller_pid == 0) {
        controller_pid = getpid();
        int priority_result = setpriority( PRIO_PROCESS, controller_pid, sim_priority + 1 );
        int controller_priority = getpriority( PRIO_PROCESS, controller_pid );

        //controller_log.exceptions( std::ofstream::failbit | std::ofstream::badbit );
        controller_log.exceptions( std::ofstream::failbit );

        try {
            controller_log.open("controller.log");
        } catch (std::ofstream::failure ex) {
            printf( "FAILED TO OPEN STREAM.\n" );  // Likely won't print to console
        }

//        if( !controller_log.is_open() )
        //            printf( "FAILED TO OPEN STREAM.\n" );

        if( priority_result < 0 ) {
            controller_log << "ERROR: Failed to set priority: " << priority_result << "\n";
        } else {
            controller_log << "Controller PID: " << controller_pid << "\n";
            controller_log << "Controller Priority: " << controller_priority << "\n";
        }


        controller();
    }
}

//-----------------------------------------------------------------------------
/**
    Suspends the controller process
*/
void suspend_controller() {
    kill( controller_pid, SIGSTOP );
}

//-----------------------------------------------------------------------------
/**
    Resumes the controller process
*/
void resume_controller() {
    kill( controller_pid, SIGCONT );
}

//-----------------------------------------------------------------------------
/**
  Initializes dynamics library
  */
void init_dynamics( ) {

}
//-----------------------------------------------------------------------------
/**
  Calls the dynamics library and runs for a given timeslice
  */
void run_dynamics( const int &quantum ) {

}
//-----------------------------------------------------------------------------
/**
    Forks off the sensor process
*/
void fork_sensor() {
    // what priority?
    sensor_pid = fork();
    if (sensor_pid < 0) {
        throw std::runtime_error( "Failed to fork sensor." ) ;
    }
    if (sensor_pid == 0) {
        // only reachable by the sensor process so start dynamics code
        sensor();
    }
}

//-----------------------------------------------------------------------------
// Coordinator - Main Loop
//-----------------------------------------------------------------------------
/**
  Coordinator/Simulator/Scheduler
  Main loop of top most process
  */
void coordinator( void ) {

    initialize_timer_pipes();

    // create the timer for controller quantum management
    int result = create_timer_controller();
    if( result != 0 ) {
        //throw std::runtime_error( "Failed to open timercontroller_to_coordinator pipe." ) ;
        printf( "ERROR: Failed to create controller process timer: code %d\n", result );
        return;
        // Note: Returning here leaves the child processes running!
    }

    char input_buffer;
    char output_buffer;

    // Suspend both to begin
    kill( controller_pid, SIGSTOP );
    //kill( dynamics_pid, SIGSTOP );

    // 0 read channel, 1 write channel
    // coordinator uses only the read from controller channel on this pipe
    close( fd_controller_to_coordinator[1] );
    // coordinator uses only the write to controller channel on this pipe
    close( fd_coordinator_to_controller[0] );

    bool simulating = false;
    bool controlling = true;

    int delay;
    while( 1 ) {
        // block waiting for controller to complete computation
        //read( fd_timercontroller_to_coordinator[0], &input_buffer, 1 );

        struct timeval tv_timenow;
        struct timespec ts;

        gettimeofday( &tv_timenow, NULL );

        clock_gettime( controller_clock, &ts );

        printf( "Controller Quantum has expired: %d\n", ts.tv_sec );


        /*
        if ( read( fd_timercontroller_to_coordinator[0], &input_buffer, 1 ) > 0 ) {
            // must determine the time differential at this point.  If controller
            // ahead of dynamics then suspend controller if controller behind
            // dynamics, controller needs to catch up

            if( _DEBUG_TO_STDOUT ) {
                printf( "Controller Quantum has expired\n" );
            }

            // for now brute force suspend
            //suspend_controller();
            // and suspend the timer

            // if x the resume controller where x is whether dynamics and controller are synch'd enough
        }
        */

        /*
        if (read(fd_sensor_to_coordinator[0], &input_buffer, 1) > 0) {
            // Sensor has published a state

            if( _DEBUG_TO_STDOUT ) {
                printf("Sensor has published a message\n");
            }
        }
        */

        run_dynamics( DEFAULT_NSEC_QUANTUM_DYNAMICS );

        /*
        if (read(fd_controller_to_coordinator[0], &input_buffer, 1) > 0) {
            // Controller has published either (1) a state request or (2) a command

            if( _DEBUG_TO_STDOUT ) {
                printf("Controller has published a message\n");
            }

            // This is just to make it send messages through the whole cycle
            //write(fd_coordinator_to_dynamics[1], &output_buffer, 1);

        }
        */

        // These are prototypes.  Don't really need this code but used to validate process control
        delay = 1;
        if( controlling ) {
            // Continue the controller
            kill( controller_pid, SIGCONT );

            // DO STUFF - ARBITRARY
            for( int i = 1; i < 1000; i++ ) delay *= i;

            // Suspend the controller
            kill( controller_pid, SIGSTOP );
            controlling = false;
            simulating = true;
        }
        /*
        delay = 1;
        if( simulating ) {
            // Continue the dynamics
            kill( dynamics_pid, SIGCONT );

            // DO STUFF - ARBITRARY
            for( int i = 1; i < 1000; i++ ) delay *= i;

            // Suspend the dynamics
            kill( dynamics_pid, SIGSTOP );
            controlling = true;
            simulating = false;
        }
        */
    }

    // cleanup - unreachable but noted for now
    close( fd_controller_to_coordinator[0] );
    close( fd_coordinator_to_controller[1] );
}

//-----------------------------------------------------------------------------
// Simulator Entry Point
//-----------------------------------------------------------------------------
/**
  Simulator Entry Point
*/
int main( int argc, char* argv[] ) {

    // Initialize process identifiers
    controller_pid = 0;
    //dynamics_pid = 0;

    // Get the sim process identifier
    sim_pid = getpid();

    // Get priority for the sim process
    sim_priority = getpriority( PRIO_PROCESS, sim_pid );

    //printf("Sim PID: %d\n", sim_pid );
    //printf("Sim Priority: %d\n", sim_priority );

    // Initialize before forking as children will inherit fd's
    initialize_ipc_pipes();

    // Fork all child processes
    fork_controller();
    fork_sensor();

    init_dynamics();

    // Transition Main Process to Coordinator main loop
    coordinator();

    return 0;
}
