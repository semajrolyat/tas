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

//-----------------------------------------------------------------------------

// Verbose
#define VERBOSE 1

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

// inline assembly for rdtsc timer
#define rdtscll(val) __asm__ __volatile__("rdtsc" : "=A" (val))

unsigned long long rdtsc_mark1;
unsigned long long rdtsc_mark2;

// inline assembly for rdtsc
inline uint64_t rdtsc() {
    uint32_t lo, hi;
    __asm__ __volatile__ (
      "xorl %%eax, %%eax\n"
      "cpuid\n"
      "rdtsc\n"
      : "=a" (lo), "=d" (hi)
      :
      : "%ebx", "%ecx");
    return (uint64_t)hi << 32 | lo;
}

/*
main()
{
    unsigned long long x;
    unsigned long long y;
    x = rdtsc();
    printf("%lld\n",x);
    y = rdtsc();
    printf("%lld\n",y);
    printf("it took this long to call printf: %lld\n",y-x);
}
*/

//-----------------------------------------------------------------------------
// reference 'man proc 5'
struct proc_stat {
    int pid;                        // %d
    char comm[256];                 // %s
    char state;                     // %c
    int ppid;                       // %d
    int pgrp;                       // %d
    int session;                    // %d
    int tty_nr;                     // %d
    int tpgid;                      // %d
    unsigned flags;                 // %u (%lu before kernel 2.6.22)
    unsigned long minflt;           // %lu
    unsigned long cminflt;          // %lu
    unsigned long majflt;           // %lu
    unsigned long cmajflt;          // %lu
    unsigned long utime;            // %lu
    unsigned long stime;            // %lu
    long cutime;                    // %ld
    long cstime;                    // %ld
    long priority;                  // %ld
    long nice;                      // %ld
    long num_threads;               // %ld
    long itrealvalue;               // %ld
    unsigned long long starttime;   // %llu (%lu before kernel 2.6)
    unsigned long vsize;            // %lu
    long rss;                       // %ld
    unsigned long rsslim;           // %lu
    unsigned long startcode;        // %lu
    unsigned long endcode;          // %lu
    unsigned long startstack;       // %lu
    unsigned long kstkesp;          // %lu
    unsigned long kstkeip;          // %lu
    unsigned long signal;           // %lu
    unsigned long blocked;          // %lu
    unsigned long sigignore;        // %lu
    unsigned long sigcatch;         // %lu
    unsigned long wchan;            // %lu
    unsigned long nswap;            // %lu
    unsigned long cnswap;           // %lu
    int exit_signal;                // %d (since kernel 2.1.22)
    int processor;          		// %d (since kernel 2.2.8)
    unsigned rt_priority;           // %u (since kernel 2.5.19) (%lu before kernel 2.6.22)
    unsigned policy;                // %u (since kernel 2.5.19) (%lu before kernel 2.6.22)
    unsigned long long delayacct_blkio_ticks;	// %llu (since kernel 2.6.18)
    unsigned long guest_time;       // %lu (since kernel 2.6.24)
    unsigned int cguest_time;       // %ld (since kernel 2.6.24)
};

static int read_proc_stat( std::string fname, struct proc_stat *s ) {

    const char *format = "%d %s %c %d %d %d %d %d %u %lu %lu %lu %lu %lu %lu %ld %ld %ld %ld %ld %ld %llu %lu %ld %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %d %d %u %u %llu %lu %ld";

    FILE *proc;
    proc = fopen( fname.c_str(),"r" );
    if( proc ) {
        if( 44 == fscanf( proc, format,
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
            &s->rsslim,
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
            &s->delayacct_blkio_ticks,
            &s->guest_time,
            &s->cguest_time
        )) {
            fclose( proc );
            return 1;
        } else {
            fclose( proc );
            return 0;
        }
    } else {
        return 0;
    }
}

static void print_proc_stat( struct proc_stat *stuff ) {
    printf( "pid = %d\n", stuff->pid );
    printf( "comm = %s\n", stuff->comm );
    printf( "state = %c\n", stuff->state );
    printf( "ppid = %d\n", stuff->ppid );
    printf( "pgrp = %d\n", stuff->pgrp );
    printf( "session = %d\n", stuff->session );
    printf( "tty_nr = %d\n", stuff->tty_nr );
    printf( "tpgid = %d\n", stuff->tpgid );
    printf( "flags = %u\n", stuff->flags );
    printf( "minflt = %lu\n", stuff->minflt );
    printf( "cminflt = %lu\n", stuff->cminflt );
    printf( "majflt = %lu\n", stuff->majflt );
    printf( "cmajflt = %lu\n", stuff->cmajflt );
    printf( "utime = %lu\n", stuff->utime );
    printf( "stime = %lu\n", stuff->stime );
    printf( "cutime = %ld\n", stuff->cutime );
    printf( "cstime = %ld\n", stuff->cstime );
    printf( "priority = %ld\n", stuff->priority );
    printf( "nice = %ld\n", stuff->nice );
    printf( "num_threads = %ld\n", stuff->num_threads );
    printf( "itrealvalue = %ld\n", stuff->itrealvalue );
    printf( "starttime = %llu\n", stuff->starttime );
    printf( "vsize = %lu\n", stuff->vsize );
    printf( "rss = %ld\n", stuff->rss );
    printf( "rsslim = %lu\n", stuff->rsslim );
    printf( "startcode = %lu\n", stuff->startcode );
    printf( "endcode = %lu\n", stuff->endcode );
    printf( "startstack = %lu\n", stuff->startstack );
    printf( "kstkesp = %lu\n", stuff->kstkesp );
    printf( "kstkeip = %lu\n", stuff->kstkeip );
    printf( "signal = %lu\n", stuff->signal );
    printf( "blocked = %lu\n", stuff->blocked );
    printf( "sigignore = %lu\n", stuff->sigignore );
    printf( "sigcatch = %lu\n", stuff->sigcatch );
    printf( "wchan = %lu\n", stuff->wchan );
    printf( "nswap = %lu\n", stuff->nswap );
    printf( "cnswap = %lu\n", stuff->cnswap );
    printf( "exit_signal = %d\n", stuff->exit_signal );
    printf( "processor = %d\n", stuff->processor );
    printf( "rt_priority = %u\n", stuff->rt_priority );
    printf( "policy = %u\n", stuff->policy );
    printf( "delayacct_blkio_ticks = %llu\n", stuff->delayacct_blkio_ticks );
    printf( "guest_time = %lu\n", stuff->guest_time );
    printf( "cguest_time = %ld\n", stuff->cguest_time );
}

//-----------------------------------------------------------------------------
// PLUGIN MANAGEMENT
//-----------------------------------------------------------------------------

void* HANDLE = NULL;
typedef void (*init_t)( int argv, char** argc );
std::list<init_t> plugins;

//-----------------------------------------------------------------------------

// attempts to read control code plugin
void read_dynamics_plugin( const char* filename ) {
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
/*
void initialize_plugins( int argc, char** argv ) {
    for( std::list<init_t>::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it)( argc, argv );
    }
}
*/
void initialize_dynamics_plugins( int argc, char** argv ) {
    for( std::list<init_t>::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it)( argc, argv );
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

    proc_stat stats;
    read_proc_stat( ss_controller_proc_filename.str(), &stats );
    interval_end_time = stats.utime;


    //rdtsc_mark2 = 0;
    rdtscll( rdtsc_mark2 );
    //rdtsc_mark2 = rdtsc();

    /*
    struct rusage ru;

    //int result = getrusage( RUSAGE_SELF, &ru );
    int result = getrusage( RUSAGE_THREAD, &ru );
    //int result = getrusage( RUSAGE_CHILDREN, &ru );

    tv_controller_quantum_end_time.tv_sec = ru.ru_stime.tv_sec + ru.ru_utime.tv_sec;
    tv_controller_quantum_end_time.tv_usec = ru.ru_stime.tv_usec + ru.ru_utime.tv_usec;
*/
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

    proc_stat stats;
    read_proc_stat( ss_controller_proc_filename.str(), &stats );
    interval_start_time = stats.utime;


    //rdtsc_mark1 = 0;
    rdtscll( rdtsc_mark1 );
    //rdtsc_mark1 = rdtsc();

    /*
    struct rusage ru;

    //int result = getrusage( RUSAGE_SELF, &ru );
    int result = getrusage( RUSAGE_THREAD, &ru );
    //int result = getrusage( RUSAGE_CHILDREN, &ru );

    tv_controller_quantum_start_time.tv_sec = ru.ru_stime.tv_sec + ru.ru_utime.tv_sec;
    tv_controller_quantum_start_time.tv_usec = ru.ru_stime.tv_usec + ru.ru_utime.tv_usec;
*/
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

    //read_dynamics_plugin( "/home/james/tas/build/libplugin.so" );
    read_dynamics_plugin( "/home/james/tas/build/libmoby_driver.so" );
    initialize_dynamics_plugins( argc, argv );

    return 0;

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

    /*
    unsigned long long x;
    unsigned long long y;
    x = rdtsc();
    printf("%lld\n",x);
    y = rdtsc();
    printf("%lld\n",y);
    printf("it took this long to call printf: %lld\n",y-x);
    */

    while( 1 ) {
        if( read( fd_timercontroller_to_coordinator[0], &input_buffer, 1 ) == -1 ) {
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
        if( VERBOSE ) printf( "Controller Quantum has expired\n" );

        // suspend controller
        suspend_controller( );
        if( VERBOSE ) printf( "Controller Suspended\n" );

        // calculate the controller interval
        //struct timeval tv_controller_now;
        //gettimeofday( &tv_controller_now, NULL );
        //int controller_interval = interval_end_time - interval_start_time;
        //if( VERBOSE ) printf( "controller_interval : %d, %06d : %d\n", tv_controller_now.tv_sec, tv_controller_now.tv_usec, controller_interval );

        unsigned long long mark_delta = rdtsc_mark2 - rdtsc_mark1;

        //printf("%lld\n",rdtsc_mark1);
        //printf("%lld\n",rdtsc_mark2);
        //printf("%lld\n",mark_delta);

        //if( VERBOSE ) printf( "mk1 : %lld; mk2 : %lld, mk_delta : %lld \n", rdtsc_mark1, rdtsc_mark2, mark_delta );
        if( VERBOSE ) printf( "mk1 : %lld; mk2 : %lld, mk_delta : %lld \n", rdtsc_mark1, rdtsc_mark2, mark_delta );

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

    if( HANDLE != NULL )
        dlclose( HANDLE );

    return 0;
}


// ------- Notes -------
// -- Rusage --
// rusage is unsatisfactory because it cannot measure the amount of time that the
// child thread runs for.  It can only record this time if the child is joined or it can record
// the time that the current process (coordinator) runs for.  The problem is that it cannot
// effectively query the process time of an active child process/thread.  It cannot therefore monitor
// the time that a controller runs for.

// -- proc --
// The granularity of proc is too small.  It can only measure at a scale determined by _SC_CLK_TCK
// which is apparantly a default of 100 on current linux and osx based distributions meaning that the
// smallest granularity available of clock would be in the centi-seconds domain where it is normal for
// controllers to operate in the milliseconds domain.  As the quantum approaches 1 centisecond, the accuracy
// gets wild.  Sometimes, the controller is indicated to run for 0 units, 1 unit, or 2 units.

// -- gettimeofday vs RDTSC --
// We are looking into RDTSC, but realistically, what is the difference between RDTSC and gettimeofday.
// getttimeofday is at least microsecond granularity.  If RDTSC cannot differentiate between system time and
// user time and between particular thread time, then it is really no different than gettimeofday.

// -- RDTSC --
// major potential issues.
// Skew between cores
// Frequency changes with clock

// high level of granularity, closer to per cycle rather than per tick
//

// -- clock_gettime --
// 10 us.  Per tick granularity

