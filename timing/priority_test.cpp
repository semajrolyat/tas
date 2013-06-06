
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

//-----------------------------------------------------------------------------

// Verbose
#define VERBOSE 1

//-----------------------------------------------------------------------------
// INTERPROCESS COMMUNICATION (IPC)
//-----------------------------------------------------------------------------

// Comm channels for IPC
int fd_timer_to_parent[2];      // channel to send data from timer to parent
int fd_wakeup_to_parent[2];     // channel to send data from wakeup to parent

//-----------------------------------------------------------------------------

/// Initializes all interprocess communication channels.  This must be done
/// before any child processes are forked or threads are spawned so that they
/// will inherit the file descriptors from the parent.
void parent_initialize_pipes( void ) {
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

    //+++++++++++++++++++

    if( pipe( fd_wakeup_to_parent ) != 0 )
        throw std::runtime_error( "Failed to open timercontroller_to_coordinator pipe." ) ;

    flags = fcntl( fd_wakeup_to_parent[0], F_GETFL, 0 );
    fcntl( fd_wakeup_to_parent[0], F_SETFL, flags );

    flags = fcntl( fd_wakeup_to_parent[1], F_GETFL, 0 );
    fcntl( fd_wakeup_to_parent[1], F_SETFL, flags | O_NONBLOCK );

}

//-----------------------------------------------------------------------------
// PROCESS MANAGEMENT
//-----------------------------------------------------------------------------

#define DEFAULT_PROCESSOR    0

//-----------------------------------------------------------------------------

// Parent process information
static pid_t parent_pid;
static int parent_priority;

// Child process information
static int child_pid;

//-----------------------------------------------------------------------------

/// Creates the child process with a priority level one step below the parent.
void parent_fork_child( void ) {

    child_pid = fork( );
    if ( child_pid < 0 ) {
        throw std::runtime_error( "Failed to fork controller." ) ;
    }
    if ( child_pid == 0 ) {
        // Note: this is the child process

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

        long long i = 0;
        while( 1 ) {
            i++;
            //printf( "(child) working\n" );
            if( i % 1000000 == 0 ) {
                printf( "(child) blocking\n" );
                // simulate blocking for validating the wakeup thread
                usleep( 1000 );
            }
        }

    }
}

//-----------------------------------------------------------------------------
// TIMER CONTROL
//-----------------------------------------------------------------------------

// Time conversions if necessary
#define NSECS_PER_SEC 1E9
#define USECS_PER_SEC 1E6

// Time budget for child process
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
clockid_t childs_clock;
sigset_t childs_rttimer_mask;

// The signal action to reference child's process signal handler
struct sigaction childs_rttimer_sigaction;

//-----------------------------------------------------------------------------

/// The signal handler for the child's timer.  Is called only when the child's
/// budget is exhausted.  Puts a message out onto the comm channel the parent
/// is listening to for notifications about this event
void parent_rttimer_child_budget_monitor_sighandler( int signum, siginfo_t *si, void *data ) {
    printf( "(timer) event\n" );
    char buf = 0;
    write( fd_timer_to_parent[1], &buf, 1 );
}

//-----------------------------------------------------------------------------

/// Blocks the timer signal if it is necessary to suppress the timer
int parent_block_childs_signal_rttimer( void ) {
    if( sigprocmask( SIG_SETMASK, &childs_rttimer_mask, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGPROCMASK;
    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------

/// Unblocks a blocked timer signal
int parent_unblock_childs_signal_rttimer( void ) {
    if( sigprocmask( SIG_UNBLOCK, &childs_rttimer_mask, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGPROCMASK;
    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------

/// Create the timer to monitor the child process
int parent_create_rttimer_child_monitor( void ) {
    timer_t timerid;
    struct sigevent sevt;
    struct itimerspec its;

    // Establish handler for timer signal
    childs_rttimer_sigaction.sa_flags = SA_SIGINFO;
    childs_rttimer_sigaction.sa_sigaction = parent_rttimer_child_budget_monitor_sighandler;
    sigemptyset( &childs_rttimer_sigaction.sa_mask );
    if( sigaction( RTTIMER_SIGNAL, &childs_rttimer_sigaction, NULL ) == -1 )
       return ERR_TIMER_FAILURE_SIGACTION;

    // intialize the signal mask
    sigemptyset( &childs_rttimer_mask );
    sigaddset( &childs_rttimer_mask, RTTIMER_SIGNAL );

    // Block timer signal temporarily
    if( parent_block_childs_signal_rttimer() != ERR_TIMER_NOERROR )
        return ERR_TIMER_FAILURE_SIGPROCMASK;

    if( clock_getcpuclockid( child_pid, &childs_clock ) != 0 )
        return ERR_TIMER_FAILURE_GETCLOCKID;

    // Create the timer
    sevt.sigev_notify = SIGEV_SIGNAL;
    sevt.sigev_signo = RTTIMER_SIGNAL;
    sevt.sigev_value.sival_ptr = &timerid;
    if( timer_create( childs_clock, &sevt, &timerid ) == -1 )
       return ERR_TIMER_FAILURE_CREATE;

    // intial delay for the timer
    its.it_value.tv_sec = DEFAULT_INITDELAY_RTTIMER_SEC;
    its.it_value.tv_nsec = DEFAULT_INITDELAY_RTTIMER_NSEC;
    // budget for the child
    its.it_interval.tv_sec = DEFAULT_BUDGET_RTTIMER_SEC;
    its.it_interval.tv_nsec = DEFAULT_BUDGET_RTTIMER_NSEC;

    // Set up the timer
    if( timer_settime( timerid, 0, &its, NULL ) == -1 )
        return ERR_TIMER_FAILURE_SETTIME;

    // NOTE: The timer is blocked at this point.  It must be unblocked in
    // the parent process for the timer to function

    return ERR_TIMER_NOERROR;
}

//-----------------------------------------------------------------------------

/// Pause the child from the parent process
void parent_suspend_child( void ) {

    kill( child_pid, SIGSTOP );
}

//-----------------------------------------------------------------------------

/// Continue the child from the parent process
void parent_resume_child( void ) {

    kill( child_pid, SIGCONT );
}

//-----------------------------------------------------------------------------

/// Initializes the child monitoring timer which determines when the child has
/// exhausted its budget
void parent_init_rttimer_child_monitor( void ) {

    if( parent_create_rttimer_child_monitor( ) != ERR_TIMER_NOERROR ) {
        printf( "Failed to create timer\n" );
        // should probably throw or exit but possible to create a zombie controller as artifact
    }

    if( VERBOSE ) printf("(parent) rttimer initialized\n");

    // unblock the timer and unblock the child process
    parent_unblock_childs_signal_rttimer( );
    parent_resume_child( );
}

//-----------------------------------------------------------------------------
// WAKEUP THREAD
//-----------------------------------------------------------------------------

// the reference to the wakeup thread
pthread_t wakeup_thread;

//-----------------------------------------------------------------------------

/// the wakeup thread function itself.  Established at the priority level one
/// below the child (therefore two below parent), this thread is unblocked when
/// the child blocks and therefore can detect when the child thread is not
/// consuming budget but is not directly suspended by the parent.
static void* wakeup_parent_on_a_blocked_child( void* arg ) {
    while( 1 ) {
        printf( "(wakeup) event\n" );
        char buf = 0;
        write( fd_wakeup_to_parent[1], &buf, 1 );

        usleep( 1000 );
    }
}

//-----------------------------------------------------------------------------

/// Spawns the wakeup thread with a priority level one below the child thread
/// (therefore two below the parent).  For more information see
/// wakeup_parent_on_a_blocked_child( void* ) above
void parent_create_wakeup_thread( void ) {

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

    pthread_create( &wakeup_thread, &attributes, wakeup_parent_on_a_blocked_child, NULL );

    printf( "wakeup created\n" );

    int policy;
    pthread_getschedparam( wakeup_thread, &policy, &param );
    printf( "wakeup priority: %d\n", param.sched_priority );

    pthread_attr_destroy( &attributes );
}

//-----------------------------------------------------------------------------
// ENTRY POINT
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

    // Must be before any forking or thread creation
    parent_initialize_pipes( );

    parent_fork_child( );
    parent_init_rttimer_child_monitor( );

    parent_create_wakeup_thread( );

    char input_buffer;
    int i = 0;

    int max_fd = std::max( fd_timer_to_parent[0], fd_wakeup_to_parent[0] );

    while( 1 ) {

        fd_set fds_children;

        FD_ZERO( &fds_children );
        FD_SET( fd_timer_to_parent[0], &fds_children );
        FD_SET( fd_wakeup_to_parent[0], &fds_children );

        // TODO: review the following code
        int result = select( max_fd + 1, &fds_children, NULL, NULL, NULL );
        if( result ) {
            if( FD_ISSET( fd_timer_to_parent[0], &fds_children ) ) {
                // timer event
                if( read( fd_timer_to_parent[0], &input_buffer, 1 ) == -1 ) {
                    switch( errno ) {
                    case EINTR:
                        // The read operation was terminated due to the receipt of a signal, and no data was transferred.
                        //printf( "error : EINTR \n" );

                        // in this case, there us a read event due to an undetermined reason causing the system to unblock,
                        // but the event is not actually triggered by the child running for the budget interval.  Instead,
                        // the read event is occurring 'immediately' after the last loop cycle, the duration is effectively zero,
                        // and the child is not actually running for any significant amount of time.  Trapping this event
                        // prevents a false positive.
                        continue;
                    default:
                        printf( "EXCEPTION: unhandled read error in main loop reading from fd_timer_to_parent" );
                        break;
                    }
                }
                parent_suspend_child( );

                printf( "(parent) received a timer event on a child\n" );
                //printf( "(parent) working\n" );

                parent_resume_child( );

            } else if( FD_ISSET( fd_wakeup_to_parent[0], &fds_children ) ) {
                // wakeup event
                if( read( fd_wakeup_to_parent[0], &input_buffer, 1 ) == -1 ) {
                    switch( errno ) {
                    case EINTR:
                        // The read operation was terminated due to the receipt of a signal, and no data was transferred.
                        //printf( "error : EINTR \n" );

                        // in this case, there us a read event due to an undetermined reason causing the system to unblock,
                        // but the event is not actually triggered by the wakeup sending a message.  Instead,
                        // the read event is occurring 'immediately' after the last loop cycle, the duration is effectively zero,
                        // and the wakeup is not actually running for any significant amount of time.  Trapping this event
                        // prevents a false positive.
                        continue;
                    default:
                        printf( "EXCEPTION: unhandled read error in main loop reading from fd_wakeup_to_parent" );
                        break;
                    }
                }

                printf( "(parent) received a wakeup from a blocked child\n" );
            }
        }

    }
}
