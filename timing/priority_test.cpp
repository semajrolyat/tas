
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

//-----------------------------------------------------------------------------

// Verbose
#define VERBOSE 1

//-----------------------------------------------------------------------------

using boost::dynamic_pointer_cast;

//-----------------------------------------------------------------------------

// Sim/Coordinator process information
static pid_t parent_pid;
static int parent_priority;

// Controller process information
static int child_pid;

int fd_timer_to_parent[2];

void initialize_pipes( void ) {
    int flags;

    // Open the controller timer to coordinator channel
    if( pipe( fd_timer_to_parent ) != 0 )
        throw std::runtime_error( "Failed to open timercontroller_to_coordinator pipe." ) ;

    // NOTE: read channel needs to be blocking to cause coordinator to block waiting for
    // timer expiration message
    flags = fcntl( fd_timer_to_parent[0], F_GETFL, 0 );
    fcntl( fd_timer_to_parent[0], F_SETFL, flags );

    // NOTE: write channel should be non-blocking
    flags = fcntl( fd_timer_to_parent[1], F_GETFL, 0 );
    fcntl( fd_timer_to_parent[1], F_SETFL, flags | O_NONBLOCK );

}

//-----------------------------------------------------------------------------
// PROCESS MANAGEMENT
//-----------------------------------------------------------------------------

#define DEFAULT_PROCESSOR    0

// The signal action to reference controller's process signal handler
struct sigaction child_rttimer_sigaction;

//-----------------------------------------------------------------------------

// The actual signal handler for the controller timer
void child_rttimer_sighandler( int signum, siginfo_t *si, void *data ) {
    char buf;
    write( fd_timer_to_parent[1], &buf, 1 );
}


//-----------------------------------------------------------------------------
// Forks off a the controller process.  Executes the external controller
// program
void spawn_childprocess( void ) {

    child_pid = fork( );
    if ( child_pid < 0 ) {
        throw std::runtime_error( "Failed to fork controller." ) ;
    }
    if ( child_pid == 0 ) {

        child_pid = getpid( );

        // restrict the cpu set s.t. controller only runs on a single processor
        cpu_set_t cpuset_mask;
        // zero out the cpu set
        CPU_ZERO( &cpuset_mask );
        // set the cpu set s.t. controller only runs on 1 processor specified by DEFAULT_PROCESSOR
        CPU_SET( DEFAULT_PROCESSOR, &cpuset_mask );
        if ( sched_setaffinity( child_pid, sizeof(cpuset_mask), &cpuset_mask ) == -1 ) {
            // there was an error setting the affinity for the controller
            // NOTE: can check errno if this occurs
            printf( "ERROR: Failed to set affinity for controller process.\n" );
            // could throw or exit or perror
        }

        int sched_policy = sched_getscheduler( child_pid );
        if ( sched_policy == -1 ) {
            // there was an error getting the scheduler policy for the controller
            // NOTE: can check errno if this occurs
            printf( "ERROR: Failed to get scheduler policy for controller process.\n" );
            // could throw or exit or perror
        } else {
            struct sched_param param;
            param.sched_priority = parent_priority - 1;
            // set the policy
            // NOTE: Round Robin is opted for in this case.  FIFO an option.
            // man pages claim that either of these policies are real-time
            sched_setscheduler( child_pid, SCHED_RR, &param );

            sched_getparam( child_pid, &param );
            printf( "child process priority: %d\n", param.sched_priority );
        }

        /*
        // testing sanity check ... TO BE COMMENTED
        int ret = sched_getaffinity( controller_pid, sizeof(cpuset_mask), &cpuset_mask );
        printf( " sched_getaffinity = %d, cpuset_mask = %08lx\n", sizeof(cpuset_mask), cpuset_mask );
        */

        if( VERBOSE ) printf( "child process initialized\n" );

        int i = 0;
        while( 1 ) {
            printf( "(child) working\n" );
            if( i++ == 100 ) {
                usleep(100);
                i = 0;
            }
        }

    }
}

//-----------------------------------------------------------------------------
// TIMER CONTROL
//-----------------------------------------------------------------------------

// Nanoseconds/Second
#define NSECS_PER_SEC 1E9
#define USECS_PER_SEC 1E6

// Quantum for Controller Timer
#define DEFAULT_QUANTUM_RTTIMER_NSEC         1E8         // 0.1 second
#define DEFAULT_QUANTUM_RTTIMER_SEC          0           // 0 second

// Initial Delay for Controller Timer
// NOTE: There has to be some delay otherwise timer is disarmed
#define DEFAULT_INITDELAY_RTTIMER_NSEC       1           // 1 nanoseconds
#define DEFAULT_INITDELAY_RTTIMER_SEC        0           // 0 second

// Error codes for timer control
#define ERR_TIMER_NOERROR                       0
#define ERR_TIMER_FAILURE_SIGACTION             1
#define ERR_TIMER_FAILURE_SIGPROCMASK           2
#define ERR_TIMER_FAILURE_CREATE                3
#define ERR_TIMER_FAILURE_SETTIME               4
#define ERR_TIMER_FAILURE_GETCLOCKID            5

//-----------------------------------------------------------------------------

// The signal identifier for the controller's real-time timer
#define RTTIMER_SIGNAL                    SIGRTMIN + 4

// timer variables
clockid_t child_clock;
sigset_t child_rttimer_mask;

//-----------------------------------------------------------------------------
// Blocks the timer signal if it is necessary to suppress the timer
int child_block_signal_rttimer( void ) {
    if( sigprocmask( SIG_SETMASK, &child_rttimer_mask, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGPROCMASK;
    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------
// Unblocks a blocked timer signal
int child_unblock_signal_rttimer( void ) {
    if( sigprocmask( SIG_UNBLOCK, &child_rttimer_mask, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGPROCMASK;
    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------
// Create the timer to monitor the controller process
int child_create_rttimer( void ) {
    timer_t timerid;
    struct sigevent sevt;
    struct itimerspec its;

    // Establish handler for timer signal
    child_rttimer_sigaction.sa_flags = SA_SIGINFO;
    child_rttimer_sigaction.sa_sigaction = child_rttimer_sighandler;
    sigemptyset( &child_rttimer_sigaction.sa_mask );
    if( sigaction( RTTIMER_SIGNAL, &child_rttimer_sigaction, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGACTION;

    // intialize the signal mask
    sigemptyset( &child_rttimer_mask );
    sigaddset( &child_rttimer_mask, RTTIMER_SIGNAL );

    // Block timer signal temporarily
    if( child_block_signal_rttimer() != ERR_TIMER_NOERROR )
        return ERR_TIMER_FAILURE_SIGPROCMASK;

    if( clock_getcpuclockid( child_pid, &child_clock ) != 0 )
        return ERR_TIMER_FAILURE_GETCLOCKID;

    // Create the timer
    sevt.sigev_notify = SIGEV_SIGNAL;
    sevt.sigev_signo = RTTIMER_SIGNAL;
    sevt.sigev_value.sival_ptr = &timerid;
    if( timer_create( child_clock, &sevt, &timerid ) == -1 )
       return ERR_TIMER_FAILURE_CREATE;

    // intial delay for the timer
    its.it_value.tv_sec = DEFAULT_INITDELAY_RTTIMER_SEC;
    its.it_value.tv_nsec = DEFAULT_INITDELAY_RTTIMER_NSEC;
    // quantum interval for the timer
    // NOTE: technically in this implementation this is a tick not the quantum
    its.it_interval.tv_sec = DEFAULT_QUANTUM_RTTIMER_SEC;
    its.it_interval.tv_nsec = DEFAULT_QUANTUM_RTTIMER_NSEC;

    // Set up the timer
    if( timer_settime( timerid, 0, &its, NULL ) == -1 )
        return ERR_TIMER_FAILURE_SETTIME;

    // NOTE: The timer is blocked at this point.  It must be unblocked in
    // the parent process for the timer to function

    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------
// Pause the child
void suspend_child( void ) {

    kill( child_pid, SIGSTOP );
}

//-----------------------------------------------------------------------------
// Continue the child
void resume_child( void ) {

    kill( child_pid, SIGCONT );
}

//-----------------------------------------------------------------------------

void child_rttimer_init( void ) {

    // unblock the controller timer and unblock the controller process
    child_unblock_signal_rttimer( );
    resume_child( );

    if( child_create_rttimer( ) != ERR_TIMER_NOERROR ) {
        printf( "Failed to create controller timer\n" );
        // should probably throw or exit but possible to create a zombie controller as artifact
    }

    if( VERBOSE ) printf("(coordinator) controller rttimer initialized\n");
}

//-----------------------------------------------------------------------------

static void* wakeup_coordinator_on_a_blocked_controller( void* arg ) {

    char buf;
    while( 1 ) {
        // write to pipe
        // sleep?  Think that must block somehow!  Write should transfer control back to coordinator though.

        buf = 1;        // some data to write for testing.  Future this will be sensor state
        write( fd_timer_to_parent[1], &buf, 1 );
    }
}

//-----------------------------------------------------------------------------
//struct thread_info *tinfo;
pthread_t pt;

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

    param.sched_priority = parent_priority - 2;

    result = pthread_attr_setschedparam( &attributes, &param);
    if( result != 0 ) {
        printf( "Failed to set schedule priority attribute for wakeup thread.\n" );
    }

    printf( "creating wakeup\n" );

    pthread_create( &pt, &attributes, wakeup_coordinator_on_a_blocked_controller, NULL );

    printf( "wakeup created\n" );

    int policy;
    pthread_getschedparam( pt, &policy, &param );
    printf( "wakeup priority: %d\n", param.sched_priority );

    pthread_attr_destroy( &attributes );
}

//-----------------------------------------------------------------------------
// Entry Point
//-----------------------------------------------------------------------------

int main( int argc, char* argv[] ) {
    // Due to realtime scheduling, et al, this program must have root access to run.
    if( geteuid() != 0 ) {
        printf( "This program requires root access.  Re-run with sudo.\nExiting.\n" );
        return 0;
    }

    parent_pid = getpid( );

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
    parent_priority = param.sched_priority;
    printf( "parent process priority: %d\n", parent_priority );

    if( VERBOSE ) printf( "parent process initialized\n" );

    spawn_childprocess( );
    initialize_pipes( );

    if( child_create_rttimer( ) != ERR_TIMER_NOERROR ) {
        printf( "Failed to create controller timer\n" );
        // should probably throw or exit but possible to create a zombie controller as artifact
    }

    // unblock the timer therefore unblock the child process
    child_unblock_signal_rttimer( );

    close( fd_timer_to_parent[1] );

    char input_buffer;
    int i = 0;
    while( 1 ) {
        if( read( fd_timer_to_parent[0], &input_buffer, 1 ) == -1 ) {
            child_block_signal_rttimer( );
            switch(errno) {
            case EINTR:
                // The read operation was terminated due to the receipt of a signal, and no data was transferred.
                //printf( "error : EINTR \n" );

                // in this case, there us a read event due to an undetermined reason causing the system to unblock,
                // but the event is not actually triggered by the controller running for the specified interval.  Instead,
                // the read event is occurring 'immediately' after the last loop cycle, the duration is effectively zero,
                // and the controller is not actually running for any significant amount of time.  Trapping this event
                // prevents a false positive.
                continue;
            default:
                printf( "EXCEPTION: unhandled read error in main loop reading from fd_timercontroller_to_coordinator\n" );
                break;
            }
        }

        for( unsigned int i = 0; i < 10; i++ ) {
            printf( "(parent) working\n" );
        }

        child_unblock_signal_rttimer( );
    }
}
