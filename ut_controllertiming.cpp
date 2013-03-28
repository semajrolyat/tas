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

//-----------------------------------------------------------------------------

// Verbose
#define VERBOSE 0

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
struct statStuff {
    int pid;			// %d
    char comm[256];		// %s
    char state;			// %c
    int ppid;			// %d
    int pgrp;			// %d
    int session;		// %d
    int tty_nr;			// %d
    int tpgid;			// %d
    unsigned long flags;	// %lu
    unsigned long minflt;	// %lu
    unsigned long cminflt;	// %lu
    unsigned long majflt;	// %lu
    unsigned long cmajflt;	// %lu
    unsigned long utime;	// %lu
    unsigned long stime; 	// %lu
    long cutime;		// %ld
    long cstime;		// %ld
    long priority;		// %ld
    long nice;			// %ld
    long num_threads;		// %ld
    long itrealvalue;		// %ld
    unsigned long starttime;	// %lu
    unsigned long vsize;	// %lu
    long rss;			// %ld
    unsigned long rlim;		// %lu
    unsigned long startcode;	// %lu
    unsigned long endcode;	// %lu
    unsigned long startstack;	// %lu
    unsigned long kstkesp;	// %lu
    unsigned long kstkeip;	// %lu
    unsigned long signal;	// %lu
    unsigned long blocked;	// %lu
    unsigned long sigignore;	// %lu
    unsigned long sigcatch;	// %lu
    unsigned long wchan;	// %lu
    unsigned long nswap;	// %lu
    unsigned long cnswap;	// %lu
    int exit_signal;		// %d
    int processor;		// %d
    unsigned long rt_priority;	// %lu
    unsigned long policy;	// %lu
    unsigned long long delayacct_blkio_ticks;	// %llu
} ;

static int readStat(std::string fname, struct statStuff *s) {

    // "d s c d d d d d u lu lu lu lu"
    //const char *format = "%d %s %c %d %d %d %d %d %lu %lu %lu %lu %lu %lu %lu %ld %ld %ld %ld %ld %ld %lu %lu %ld %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %d %d %lu %lu %llu";
    const char *format = "%d %s %c %d %d %d %d %d %u %lu %lu %lu %lu %lu %lu %ld %ld %ld %ld %ld %ld %llu %lu %ld %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %d %d %u %u %llu"; // new ones" %lu %ld"

    FILE *proc;
    proc = fopen( fname.c_str(),"r");
    if (proc) {
        if (42==fscanf(proc, format,
        &s->pid,
        s->comm,
        &s->state,
        &s->ppid,
        &s->pgrp,
        &s->session,
        &s->tty_nr,
        &s->tpgid,
        &s->flags,
        &s->minflt,
        &s->cminflt,
        &s->majflt,
        &s->cmajflt,
        &s->utime,
        &s->stime,
        &s->cutime,
        &s->cstime,
        &s->priority,
        &s->nice,
        &s->num_threads,
        &s->itrealvalue,
        &s->starttime,
        &s->vsize,
        &s->rss,
        &s->rlim,
        &s->startcode,
        &s->endcode,
        &s->startstack,
        &s->kstkesp,
        &s->kstkeip,
        &s->signal,
        &s->blocked,
        &s->sigignore,
        &s->sigcatch,
        &s->wchan,
        &s->nswap,
        &s->cnswap,
        &s->exit_signal,
        &s->processor,
        &s->rt_priority,
        &s->policy,
        &s->delayacct_blkio_ticks
    )) {
       fclose(proc);
       return 1;
        } else {
       fclose(proc);
       return 0;
        }
     } else {
    return 0;
     }
}

static void printStat(struct statStuff *stuff) {
    printf("pid = %d\n", stuff->pid);
    printf("comm = %s\n", stuff->comm);
    printf("state = %c\n", stuff->state);
    printf("ppid = %d\n", stuff->ppid);
    printf("pgrp = %d\n", stuff->pgrp);
    printf("session = %d\n", stuff->session);
    printf("tty_nr = %d\n", stuff->tty_nr);
    printf("tpgid = %d\n", stuff->tpgid);
    printf("flags = %lu\n", stuff->flags);
    printf("minflt = %lu\n", stuff->minflt);
    printf("cminflt = %lu\n", stuff->cminflt);
    printf("majflt = %lu\n", stuff->majflt);
    printf("cmajflt = %lu\n", stuff->cmajflt);
    printf("utime = %lu\n", stuff->utime);
    printf("stime = %lu\n", stuff->stime);
    printf("cutime = %ld\n", stuff->cutime);
    printf("cstime = %ld\n", stuff->cstime);
    printf("priority = %ld\n", stuff->priority);
    printf("nice = %ld\n", stuff->nice);
    printf("num_threads = %ld\n", stuff->num_threads);
    printf("itrealvalue = %ld\n", stuff->itrealvalue);
    printf("starttime = %lu\n", stuff->starttime);
    printf("vsize = %lu\n", stuff->vsize);
    printf("rss = %ld\n", stuff->rss);
    printf("rlim = %lu\n", stuff->rlim);
    printf("startcode = %lu\n", stuff->startcode);
    printf("endcode = %lu\n", stuff->endcode);
    printf("startstack = %lu\n", stuff->startstack);
    printf("kstkesp = %lu\n", stuff->kstkesp);
    printf("kstkeip = %lu\n", stuff->kstkeip);
    printf("signal = %lu\n", stuff->signal);
    printf("blocked = %lu\n", stuff->blocked);
    printf("sigignore = %lu\n", stuff->sigignore);
    printf("sigcatch = %lu\n", stuff->sigcatch);
    printf("wchan = %lu\n", stuff->wchan);
    printf("nswap = %lu\n", stuff->nswap);
    printf("cnswap = %lu\n", stuff->cnswap);
    printf("exit_signal = %d\n", stuff->exit_signal);
    printf("processor = %d\n", stuff->processor);
    printf("rt_priority = %lu\n", stuff->rt_priority);
    printf("policy = %lu\n", stuff->policy);
    printf("delayacct_blkio_ticks = %llu\n", stuff->delayacct_blkio_ticks);
}



//-----------------------------------------------------------------------------
// PLUGIN MANAGEMENT
//-----------------------------------------------------------------------------

void* HANDLE = NULL;
typedef void (*init_t)( void* );
std::list<init_t> plugins;

//-----------------------------------------------------------------------------

// attempts to read control code plugin
void read_plugin( const char* filename ) {
    // attempt to read the file
    HANDLE = dlopen( filename, RTLD_LAZY );
    if ( !HANDLE ) {
        std::cerr << " failed to read plugin from " << filename << std::endl;
        std::cerr << "  " << dlerror( ) << std::endl;
        exit( -1 );
    }

    // attempt to load the initializer
    dlerror( );
    plugins.push_back( (init_t) dlsym(HANDLE, "init") );
    const char* dlsym_error = dlerror( );
    if ( dlsym_error ) {
        std::cerr << " warning: cannot load symbol 'init' from " << filename << std::endl;
        std::cerr << "        error follows: " << std::endl << dlsym_error << std::endl;
        plugins.pop_back( );
    }
}

void initialize_plugins( ) {
    for( std::list<init_t>::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it)(NULL);
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
#define DEFAULT_CONTROLLER_PROGRAM      "controller"
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

        // exit on fail to exec
        _exit( 1 );
    }
}

std::stringstream ss_controller_proc_filename("");
int interval_start_time;
int interval_end_time;

//-----------------------------------------------------------------------------
// Block/wait the controller process
// Note: Timer signal will be blocked if the controller is blocked due to
// construction of the interface of the signal handler.  Therefore, using
// this method effectively pauses the timer as well so this function is
// a catch all for managing signal suspension and process suspension
void suspend_controller( ) {
    kill( controller_pid, SIGSTOP );

    // NOTE : gettimeofday is not an accurate metric.  Need process time!
    //gettimeofday( &tv_controller_quantum_end_time, NULL );

    statStuff stats;
    readStat( ss_controller_proc_filename.str(), &stats );
    interval_end_time = stats.utime;

    /*
    struct rusage ru;

    //int result = getrusage( RUSAGE_SELF, &ru );
    int result = getrusage( RUSAGE_THREAD, &ru );
    //int result = getrusage( RUSAGE_CHILDREN, &ru );

    tv_controller_quantum_end_time.tv_sec = ru.ru_stime.tv_sec + ru.ru_utime.tv_sec;
    tv_controller_quantum_end_time.tv_usec = ru.ru_stime.tv_usec + ru.ru_utime.tv_usec;

    /*
    if( result != 0 ) {
        printf( "tv_controller_quantum_end_time query failed" );
    } else {
        printf( "tv_controller_quantum_end_time : %d, %06d\n", tv_controller_quantum_end_time.tv_sec, tv_controller_quantum_end_time.tv_usec );
    }
    */

}

//-----------------------------------------------------------------------------
// Unpause the controller process
void resume_controller( ) {
    // NOTE : gettimeofday is not an accurate metric.  Need process time!
    //gettimeofday( &tv_controller_quantum_start_time, NULL );

    statStuff stats;
    readStat( ss_controller_proc_filename.str(), &stats );
    interval_start_time = stats.utime;

    /*
    struct rusage ru;

    //int result = getrusage( RUSAGE_SELF, &ru );
    int result = getrusage( RUSAGE_THREAD, &ru );
    //int result = getrusage( RUSAGE_CHILDREN, &ru );

    tv_controller_quantum_start_time.tv_sec = ru.ru_stime.tv_sec + ru.ru_utime.tv_sec;
    tv_controller_quantum_start_time.tv_usec = ru.ru_stime.tv_usec + ru.ru_utime.tv_usec;

    /*
    if( result != 0 ) {
        printf( "tv_controller_quantum_start_time query failed" );
    } else {
        printf( "tv_controller_quantum_start_time : %d, %06d\n", tv_controller_quantum_start_time.tv_sec, tv_controller_quantum_start_time.tv_usec );
    }
*/
    kill( controller_pid, SIGCONT );
}

//-----------------------------------------------------------------------------

// How long to consume resourses via intensive computation
#define ITS_TO_WAIT             850000
//#define ITS_TO_WAIT             280000

//-----------------------------------------------------------------------------
// Computational intensive function to consume resources and time to simulate
// running dynamics
void run_dynamics( int dt ) {

    // time suck
    double x = 1.1;
    for( int i = 0; i < ITS_TO_WAIT; i++ ) {
        x *= x;
    }
}

//-----------------------------------------------------------------------------
// Simulator Entry Point
//-----------------------------------------------------------------------------

int main( int argc, char* argv[] ) {

    read_plugin( "/home/james/tas/build/libplugin.so" );
    initialize_plugins();

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

    char input_buffer;

    // unblock the controller timer and unblock the controller process
    unblock_controller_timer( );
    resume_controller( );

    ss_controller_proc_filename << "/proc" << "/" << controller_pid << "/stat";

    while( 1 ) {
        if( read( fd_timercontroller_to_coordinator[0], &input_buffer, 1 ) == -1 ) {
            switch(errno) {
            case EINTR:
                // The read operation was terminated due to the receipt of a signal, and no data was transferred.
                //printf( "error : EINTR \n" );
                continue;
            default:
                printf( "EXCEPTION: unhandled read error in main loop reading from fd_timercontroller_to_coordinator\n" );
                break;
            }
        }
        if( VERBOSE ) printf( "Controller Quantum has expired\n" );

        // suspend controller
        suspend_controller( );
        if( VERBOSE ) printf( "Controller Suspended\n" );

        // calculate the controller interval
        struct timeval tv_controller_now;
        gettimeofday( &tv_controller_now, NULL );
        int controller_interval = interval_end_time - interval_start_time;
        if( VERBOSE ) printf( "controller_interval : %d, %06d : %d\n", tv_controller_now.tv_sec, tv_controller_now.tv_usec, controller_interval );

        /*
        tv_controller_quantum_interval.tv_sec = tv_controller_quantum_end_time.tv_sec - tv_controller_quantum_start_time.tv_sec;
        tv_controller_quantum_interval.tv_usec = tv_controller_quantum_end_time.tv_usec - tv_controller_quantum_start_time.tv_usec;

        if( VERBOSE ) printf( "tv_controller_quantum_interval : %d, %06d\n", tv_controller_quantum_interval.tv_sec, tv_controller_quantum_interval.tv_usec );
        */
        // Run Dynamics
        int dt = 1;
        run_dynamics( dt );

        // Resume Controller
        resume_controller( );
        if( VERBOSE ) printf( "Controller Resumed\n" );
    }

    return 0;
}
