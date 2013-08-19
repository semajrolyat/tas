/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

time.h
-----------------------------------------------------------------------------*/

#ifndef _TIME_H_
#define _TIME_H_

#include <tas/tas.h>
#include <tas/error.h>
#include <tas/log.h>

typedef double Real;

//-----------------------------------------------------------------------------

#define NSECS_PER_SEC 1E9
#define USECS_PER_SEC 1E6

//-----------------------------------------------------------------------------

/// Conversion function from a timeval to a floating point representing seconds
inline Real timeval_to_real( const struct timeval& tv ) {
    return (Real) tv.tv_sec + (Real) tv.tv_usec / (Real) USECS_PER_SEC;
}

//-----------------------------------------------------------------------------
inline Real timespec_to_real( const struct timespec& ts ) {
    return (Real) ts.tv_sec + (Real) ts.tv_nsec / (Real) NSECS_PER_SEC;
}

//-----------------------------------------------------------------------------

inline double cycles_to_seconds( const unsigned long long& cycles, const unsigned long long& cpu_hz ) {
    return (double)cycles / (double)cpu_hz;
}

//-----------------------------------------------------------------------------

inline unsigned long long seconds_to_cycles( const double& seconds, const unsigned long long& cpu_hz ) {
    return (unsigned long long)(seconds * (double)cpu_hz);
}

//-----------------------------------------------------------------------------

inline unsigned long long nanoseconds_to_cycles( const long& ns, const unsigned long long& cpu_hz ) {
    return ((unsigned long long)ns * (unsigned long long)cpu_hz)/(unsigned long long)NSECS_PER_SEC;
}

//-----------------------------------------------------------------------------

inline unsigned long long cycles_to_nanoseconds( const unsigned long long& cycles, const unsigned long long& cpu_hz ) {
    return (cycles * (unsigned long long) NSECS_PER_SEC) / cpu_hz;
}

//-----------------------------------------------------------------------------

struct timespec subtract_ts( const struct timespec& ts1, const struct timespec& ts2 ) {
    struct timespec result;

    if( ts1.tv_sec < ts2.tv_sec || ( ts1.tv_sec == ts2.tv_sec && ts1.tv_nsec <= ts2.tv_nsec) ) {
        result.tv_sec = 0 ;
        result.tv_nsec = 0 ;
    } else if( ts1.tv_sec == ts2.tv_sec ) {
        // ts1.tv_sec == ts2.tv_sec && ts1.tv_nsec > ts2.tv_nsec
        result.tv_sec = 0;
        result.tv_nsec = ts1.tv_nsec - ts2.tv_nsec;
    } else if( ts1.tv_nsec >= ts2.tv_nsec ) {
        // ts1.tv_sec > ts2.tv_sec && ts1.tv_nsec >= ts2.tv_nsec
        result.tv_sec = ts1.tv_sec - ts2.tv_sec;
        result.tv_nsec = ts1.tv_nsec - ts2.tv_nsec;
    } else {
        // ts1.tv_sec > ts2.tv_sec && ts1.tv_nsec < ts2.tv_nsec
        result.tv_sec = ts1.tv_sec - ts2.tv_sec - 1;
        result.tv_nsec = ts1.tv_nsec + NSECS_PER_SEC - ts2.tv_nsec;
    }
    return result;
}

// inline assembly for rdtsc timer
#define rdtscll(val) __asm__ __volatile__("rdtsc" : "=A" (val))


//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

struct timestamp_t {
    unsigned long long t_cycle;
};


struct realtime_t {
    Real seconds;
};




//-----------------------------------------------------------------------------

// arbitrary size : buffer > 100s * 1000Hz
//#define TIMESTAMP_BUFFER_SIZE 1000000

#define TIMESTAMP_BUFFER_SIZE 500000

class timestamp_buffer_c {
private:
    //timestamp_t buffer[TIMESTAMP_BUFFER_SIZE];
    unsigned long long buffer[TIMESTAMP_BUFFER_SIZE];

    unsigned int cursor;
    unsigned long long cpu_hz;
    std::string filename;
public:
    timestamp_buffer_c( void ) {
        cursor = 0;
        cpu_hz = 0;
    }
    timestamp_buffer_c( const unsigned long long& cpu_hz, std::string filename ) {
        cursor = 0;
        this->cpu_hz = cpu_hz;
        this->filename = filename;
    }

    virtual ~timestamp_buffer_c( void ) { }

    /// initialize
    error_t open( void ) {
        return ERROR_NONE;
    }

    /// write to disk
    error_t close( void ) {
        char strbuffer[256];

        //printf( "closing timestamp log\n" );

        if( cursor == 0 ) return ERROR_NONE;

        log_c log = log_c( filename );

        if( log.open( ) != LOG_ERROR_NONE )
            return ERROR_FAILED;

        //printf( "writing timestamp log to disk\n" );
        //printf( "records: %d\n", cursor );

        sprintf( strbuffer, "cpu_hz: %lld\n", cpu_hz );
        log.write( strbuffer );

        sprintf( strbuffer, "cycles\tseconds\n" );
        log.write( strbuffer );

        for( unsigned int i = 0; i < cursor; i++ ) {

	    double seconds = cycles_to_seconds( buffer[i], cpu_hz );
            sprintf( strbuffer, "%lld\t%10.9f\n", buffer[i], seconds );
            //buffer[i].seconds = cycles_to_seconds( buffer[i].cycles, cpu_hz );
            //sprintf( strbuffer, "%lld\t%10.9f\n", buffer[i].cycles, buffer[i].seconds );
            log.write( strbuffer );
        }

        log.close( );

        return ERROR_NONE;
    }

    /*
    error_t write( const timestamp_t& ts ) {
        assert( cursor < TIMESTAMP_BUFFER_SIZE - 1 );

        buffer[cursor].cycles = ts.cycles;
        buffer[cursor].seconds = ts.seconds;
        cursor++;

        return ERROR_NONE;
    }

    error_t write( const unsigned long long& cycles, const double& seconds ) {
        assert( cursor < TIMESTAMP_BUFFER_SIZE - 1 );

        buffer[cursor].cycles = cycles;
        buffer[cursor].seconds = seconds;
        cursor++;

        return ERROR_NONE;
    }
    */

    error_t write( const unsigned long long& cycles ) {
        assert( cursor < TIMESTAMP_BUFFER_SIZE - 1 );

        buffer[cursor] = cycles;
        //buffer[cursor].cycles = cycles;
        cursor++;

        return ERROR_NONE;
    }

};


//-----------------------------------------------------------------------------
// Timing Functions
//-----------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------

// The signal identifier for the controller's real-time timer
#define RTTIMER_SIGNAL                          SIGRTMIN + 4

//-----------------------------------------------------------------------------
/// Error codes for timer control
enum timer_err_e {
    TIMER_ERROR_NONE = 0,
    TIMER_ERROR_SIGACTION,
    TIMER_ERROR_SIGPROCMASK,
    TIMER_ERROR_CREATE,
    TIMER_ERROR_SETTIME,
    TIMER_ERROR_GETCLOCKID
};
//-----------------------------------------------------------------------------

///*
// Note: Assumes that there is no overflow of seconds
struct timespec add_timespecs( const struct timespec& a, const struct timespec& b ) {
    struct timespec result;
    int s = a.tv_sec + b.tv_sec;
    int ns = a.tv_nsec + b.tv_nsec;
    int nsof = ns - 1000000000;

    if( nsof >= 0 ) {
        result.tv_sec = s + 1;
        result.tv_nsec = nsof;
    } else {
        result.tv_sec = s;
        result.tv_nsec = ns;
    }
    return result;
}

// Note: results are too high at this point.  Revisit later
// TODO : determine more reliable calibration code
//Note: Should probably repeat this several times and average the results
// Note: querying /proc/cpuinfo is no more accurate than KHz so 1 ms ideally would
// give the same result, but there is implicit delay due to operations
unsigned long long calibrate_cycles_per_second( void ) {
    struct timespec tspec_time, tspec_period, tspec_end;
    unsigned long long ts_start, ts_end;

    printf( "Calibrating CPU speed\n" );
///*
    // ++++ Method 1.  Absolute time ++++

    tspec_period.tv_sec = 1;
    tspec_period.tv_nsec = 0;
    //tspec_period.tv_sec = 0;
    //tspec_period.tv_nsec = 1000000;

    clock_gettime( CLOCK_MONOTONIC, &tspec_time );
    tspec_end = add_timespecs( tspec_time, tspec_period );

    rdtscll( ts_start );

    clock_nanosleep( CLOCK_MONOTONIC, TIMER_ABSTIME, &tspec_end, NULL );

    rdtscll( ts_end );

    return ( ts_end - ts_start );
    //return ( ts_end - ts_start ) * 1000;
//*/
/*
    // ++++ Method 2.  Relative time ++++
    tspec_period.tv_sec = 1;
    tspec_period.tv_nsec = 0;

    rdtscll( ts_start );

    clock_nanosleep( CLOCK_MONOTONIC, 0, &tspec_period, NULL );

    rdtscll( ts_end );

    return ( ts_end - ts_start );
*/
}
//*/


typedef void ( *timer_sighandler )( int signum, siginfo_t *si, void *data );

class timer_c {
public:
    timer_c( void ) {
        first_arming = true;
        agg_error = 0;
        last_overrun = 0;
        ts_prev_arm = 0;
    }
    virtual ~timer_c( void ) { }

    sigset_t rttimer_mask;
    struct sigaction rttimer_sigaction;
    timer_t rttimer_id;

    bool first_arming;
    long long agg_error;
    unsigned long long last_overrun;
    unsigned long long ts_prev_arm;

    /// Error codes for timer control
    enum timer_err_e {
        TIMER_ERROR_NONE = 0,
        TIMER_ERROR_SIGACTION,
        TIMER_ERROR_SIGPROCMASK,
        TIMER_ERROR_CREATE,
        TIMER_ERROR_SETTIME,
        TIMER_ERROR_GETCLOCKID
    };

    /// Blocks the timer signal if it is necessary to suppress the timer
    timer_err_e block( void ) {
        if( sigprocmask( SIG_SETMASK, &rttimer_mask, NULL ) == -1 )
            return TIMER_ERROR_SIGPROCMASK;
        return TIMER_ERROR_NONE;
    }

    //-----------------------------------------------------------------------------

    /// Unblocks a blocked timer signal
    timer_err_e unblock( void ) {
        if( sigprocmask( SIG_UNBLOCK, &rttimer_mask, NULL ) == -1 )
            return TIMER_ERROR_SIGPROCMASK;
        return TIMER_ERROR_NONE;
    }

    /// Creates a high-resolution real-time timer to monitor the controller process
    timer_err_e create( timer_sighandler sighandler, int signum ) {
        struct sigevent sevt;
        //struct itimerspec its;

        // Establish handler for timer signal
        rttimer_sigaction.sa_flags = SA_SIGINFO;
        rttimer_sigaction.sa_sigaction = sighandler;
        sigemptyset( &rttimer_sigaction.sa_mask );
        if( sigaction( signum, &rttimer_sigaction, NULL ) == -1)
            return TIMER_ERROR_SIGACTION;

        // intialize the signal mask
        sigemptyset( &rttimer_mask );
        sigaddset( &rttimer_mask, signum );

        sevt.sigev_notify = SIGEV_SIGNAL;
        sevt.sigev_signo = signum;
        sevt.sigev_value.sival_ptr = &rttimer_id;
        if( timer_create( CLOCK_MONOTONIC, &sevt, &rttimer_id ) == -1 )
        //if( timer_create( CLOCK_REALTIME, &sevt, &rttimer_id ) == -1 )
           return TIMER_ERROR_CREATE;

        return TIMER_ERROR_NONE;
    }


    timer_err_e arm( const unsigned long long& ts_req, unsigned long long& ts_arm, const unsigned long long& period_nsec, const unsigned long long& cpu_speed_hz ) {
        struct itimerspec its;
        unsigned long long ts_now;

        // sanity check.  Not supporting controllers that run at 1 Hz or slower or faster than 1ns.
        //assert( CONTROLLER_HZ > 1 && CONTROLLER_HZ <= NSECS_PER_SEC );

        unsigned long long ts_err = 0;
        unsigned long long ns_err = 0;

        unsigned long long period = period_nsec;

        rdtscll( ts_now );

        if( first_arming ) {
            // cannot compute err correction
            first_arming = false;
            } else {
            ts_err = ts_now - ts_prev_arm;
            ns_err = cycles_to_nanoseconds( ts_err, cpu_speed_hz );
            long long error = (long long) period - (long long) ns_err;

            agg_error += error;
            period += agg_error;
        }

        its.it_interval.tv_sec = 0;
        its.it_interval.tv_nsec = 0;
        its.it_value.tv_sec = 0;

        its.it_value.tv_nsec = (unsigned long long)(period);

        ts_arm = ts_req;
        ts_prev_arm = ts_now;

        if( timer_settime( rttimer_id, 0, &its, NULL ) == -1 ) {
            //printf( "dts: %lld, dns: %lld, nsec: %d\n", dts, dns, its.it_value.tv_nsec );
            return TIMER_ERROR_SETTIME;
        }

        return TIMER_ERROR_NONE;
    }

};



/*
// reference 'man proc 5'
struct proc_stat_t {
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
    long cguest_time;       	    // %ld (since kernel 2.6.24)
};

//-----------------------------------------------------------------------------

enum proc_stat_err_e {
    PROC_STAT_ERR_NONE,
    PROC_STAT_ERR_OPEN,
    PROC_STAT_ERR_READ,
    PROC_STAT_ERR_CLOSE
};


//-----------------------------------------------------------------------------

static proc_stat_err_e read_proc_stat( const char* filename, struct proc_stat_t *s ) {

    const char *format = "%d %s %c %d %d %d %d %d %u %lu %lu %lu %lu %lu %lu %ld %ld %ld %ld %ld %ld %llu %lu %ld %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %d %d %u %u %llu %lu %ld";
    proc_stat_err_e err = PROC_STAT_ERR_NONE;

    FILE* fproc = fopen( filename, "r" );
    if( fproc == 0 ) 	return PROC_STAT_ERR_OPEN;

    if( 44 != fscanf( fproc, format,
                      &s->pid,
                      &s->comm,
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
                      ))
        err = PROC_STAT_ERR_READ;

    fclose( fproc );
    return err;
}

//-----------------------------------------------------------------------------
static proc_stat_err_e get_proc_stat( const int& cpuid, struct proc_stat_t *s ) {
    std::string str;
    // TODO : build string
    return read_proc_stat( str.c_str( ), s );
}

//-----------------------------------------------------------------------------

static void print_proc_stat( struct proc_stat_t *s ) {
    printf( "pid = %d\n", s->pid );
    printf( "comm = %s\n", s->comm );
    printf( "state = %c\n", s->state );
    printf( "ppid = %d\n", s->ppid );
    printf( "pgrp = %d\n", s->pgrp );
    printf( "session = %d\n", s->session );
    printf( "tty_nr = %d\n", s->tty_nr );
    printf( "tpgid = %d\n", s->tpgid );
    printf( "flags = %u\n", s->flags );
    printf( "minflt = %lu\n", s->minflt );
    printf( "cminflt = %lu\n", s->cminflt );
    printf( "majflt = %lu\n", s->majflt );
    printf( "cmajflt = %lu\n", s->cmajflt );
    printf( "utime = %lu\n", s->utime );
    printf( "stime = %lu\n", s->stime );
    printf( "cutime = %ld\n", s->cutime );
    printf( "cstime = %ld\n", s->cstime );
    printf( "priority = %ld\n", s->priority );
    printf( "nice = %ld\n", s->nice );
    printf( "num_threads = %ld\n", s->num_threads );
    printf( "itrealvalue = %ld\n", s->itrealvalue );
    printf( "starttime = %llu\n", s->starttime );
    printf( "vsize = %lu\n", s->vsize );
    printf( "rss = %ld\n", s->rss );
    printf( "rsslim = %lu\n", s->rsslim );
    printf( "startcode = %lu\n", s->startcode );
    printf( "endcode = %lu\n", s->endcode );
    printf( "startstack = %lu\n", s->startstack );
    printf( "kstkesp = %lu\n", s->kstkesp );
    printf( "kstkeip = %lu\n", s->kstkeip );
    printf( "signal = %lu\n", s->signal );
    printf( "blocked = %lu\n", s->blocked );
    printf( "sigignore = %lu\n", s->sigignore );
    printf( "sigcatch = %lu\n", s->sigcatch );
    printf( "wchan = %lu\n", s->wchan );
    printf( "nswap = %lu\n", s->nswap );
    printf( "cnswap = %lu\n", s->cnswap );
    printf( "exit_signal = %d\n", s->exit_signal );
    printf( "processor = %d\n", s->processor );
    printf( "rt_priority = %u\n", s->rt_priority );
    printf( "policy = %u\n", s->policy );
    printf( "delayacct_blkio_ticks = %llu\n", s->delayacct_blkio_ticks );
    printf( "guest_time = %lu\n", s->guest_time );
    printf( "cguest_time = %ld\n", s->cguest_time );
}
*/
//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------


#endif // _TIME_H_
