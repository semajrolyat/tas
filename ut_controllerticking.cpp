/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

Extension of ut_controllerexec.cpp prototyping.  This test validates
interprocess communication and process manangement over an external controller
program
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

#define _DEBUG_TO_STDOUT 1

//-----------------------------------------------------------------------------

static int sim_pid;
static int sim_priority;

static int controller_pid;

std::ofstream controller_log;

//-----------------------------------------------------------------------------
// INTERPROCESS COMMUNICATION
//-----------------------------------------------------------------------------

int fd_timercontroller_to_coordinator[2];

//-----------------------------------------------------------------------------

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

#define NSECS_PER_SEC 1E9

// Quantum for Controller Timer
//#define DEFAULT_NSEC_QUANTUM_CONTROLLER         3E7         // 30 milliseconds
//#define DEFAULT_SEC_QUANTUM_CONTROLLER          1           // 1 second
#define DEFAULT_QUANTUM_CONTROLLER_NSEC         1E8         // 0.1 second
#define DEFAULT_QUANTUM_CONTROLLER_SEC          0           // 0 second

// Initial Delay for Controller Timer
// NOTE: There has to be some delay otherwise timer is disarmed
#define DEFAULT_INITDELAY_CONTROLLER_NSEC       1           // 1 nanoseconds
#define DEFAULT_INITDELAY_CONTROLLER_SEC        0           // 0 second

//-----------------------------------------------------------------------------

struct sigaction sigaction_contoller;

//-----------------------------------------------------------------------------

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

#define SIG_TIMER_CONTROLLER                    SIGRTMIN + 4

clockid_t controller_clock;
sigset_t controller_timer_mask;

//-----------------------------------------------------------------------------

int block_controller_timer() {
    if( sigprocmask( SIG_SETMASK, &controller_timer_mask, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGPROCMASK;
    return ERR_TIMER_NOERROR;
}

int unblock_controller_timer() {
    if( sigprocmask( SIG_UNBLOCK, &controller_timer_mask, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGPROCMASK;
    return ERR_TIMER_NOERROR;
}

int create_controller_timer() {
    timer_t timerid;
    struct sigevent sevt;
    struct itimerspec its;

    // Establish handler for timer signal
    sigaction_contoller.sa_flags = SA_SIGINFO;
    sigaction_contoller.sa_sigaction = sighandler_timer_controller;
    sigemptyset( &sigaction_contoller.sa_mask );
    if( sigaction( SIG_TIMER_CONTROLLER, &sigaction_contoller, NULL ) == -1 )
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

#define DEFAULT_CONTROLLER_PROGRAM      "controller"

void fork_controller() {

    controller_pid = fork();
    if (controller_pid < 0) {
        throw std::runtime_error( "Failed to fork controller." ) ;
    }
    if (controller_pid == 0) {
        controller_pid = getpid();

        int priority_result = setpriority( PRIO_PROCESS, controller_pid, sim_priority + 1 );
        int controller_priority = getpriority( PRIO_PROCESS, controller_pid );

        // execute the controller program
        execl( DEFAULT_CONTROLLER_PROGRAM, "controller", 0 );

        // exit on fail to exec
        _exit(1);
    }
}

//-----------------------------------------------------------------------------
void suspend_controller() {
    kill( controller_pid, SIGSTOP );
}

//-----------------------------------------------------------------------------
void resume_controller() {
    kill( controller_pid, SIGCONT );
}


//-----------------------------------------------------------------------------
// Simulator Entry Point
//-----------------------------------------------------------------------------
/**
  Simulator Entry Point
*/
int main( int argc, char* argv[] ) {

    controller_pid = 0;

    sim_pid = getpid();
    sim_priority = getpriority( PRIO_PROCESS, sim_pid );

    // fork the controller and immediately block it
    // NOTE: the controller process may need time to run through initialization
    // which may be delayed by blocking s.t. the initial quantum may get out of
    // step anyway.
    fork_controller();
    suspend_controller();

    // controller has no need to know about these channels
    // comm done completely in the parent process hence init after fork
    initialize_timer_pipes();

    if( create_controller_timer() != ERR_TIMER_NOERROR) {
        printf( "Failed to create controller timer\n" );
        // should probably throw or exit but possible to create a zombie controller
    }

    char input_buffer;

    int TICKS_PER_QUANTUM = 10;
    int num_ticks_this_quantum = 0;

    // unblock the controller timer and unblock the controller process
    unblock_controller_timer();
    resume_controller();

    while(1) {
        read( fd_timercontroller_to_coordinator[0], &input_buffer, 1 );
        num_ticks_this_quantum++;

        if( num_ticks_this_quantum == TICKS_PER_QUANTUM ) {
            printf( "Controller Quantum has expired\n" );
            num_ticks_this_quantum = 0;

            // suspend controller
            suspend_controller();
        } else {
            printf( "Controller Tick\n" );
        }
    }

    return 0;
}
