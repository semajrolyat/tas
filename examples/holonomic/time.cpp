/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

time.h
-----------------------------------------------------------------------------*/

#include "time.h"


//-----------------------------------------------------------------------------
timestamp_t generate_timestamp( void ) {
  timestamp_t ts;
  rdtscll( ts.cycle );
  return ts;
}

//-----------------------------------------------------------------------------
// TIME UTILITIES
//-----------------------------------------------------------------------------

/// Conversion from POSIX timeval to floating point seconds
double timeval_to_real( const struct timeval& tv ) {
    return (double) tv.tv_sec + (double) tv.tv_usec / (double) USECS_PER_SEC;
}

//-----------------------------------------------------------------------------

/// Conversion from POSIX timespec to floating point seconds
double timespec_to_real( const struct timespec& ts ) {
    return (double) ts.tv_sec + (double) ts.tv_nsec / (double) NSECS_PER_SEC;
}

//-----------------------------------------------------------------------------

/// Conversion from POSIX timeval to realtime_t type
realtime_t timeval_to_realtime( const struct timeval& tv ) {
    realtime_t rt;
    rt.seconds =  timeval_to_real( tv );
    return rt;
}

//-----------------------------------------------------------------------------

/// Conversion from POSIX timespec to realtime_t type
realtime_t timespec_to_realtime( const struct timespec& ts ) {
    realtime_t rt;
    rt.seconds =  timespec_to_real( ts );
    return rt;
}

//-----------------------------------------------------------------------------

/// Conversion from whole cycles to floating point seconds
double cycles_to_seconds( const unsigned long long& cycles, const unsigned long long& cpu_hz ) {
    return (double)cycles / (double)cpu_hz;
}

//-----------------------------------------------------------------------------

/// Conversion from timestamp_t type to floating point seconds
double timestamp_to_seconds( const timestamp_t& ts, const unsigned long long& cpu_hz ) {
    return cycles_to_seconds( ts.cycle, cpu_hz );
}

//-----------------------------------------------------------------------------

/// Conversion from timestamp_t type to realtime_t type
realtime_t timestamp_to_realtime( const timestamp_t& ts, const unsigned long long& cpu_hz ) {
    realtime_t rt;
    rt.seconds = cycles_to_seconds( ts.cycle, cpu_hz );
    return rt;
}

//-----------------------------------------------------------------------------

/// Conversion from floating point seconds to whole cycles
unsigned long long seconds_to_cycles( const double& seconds, const unsigned long long& cpu_hz ) {
    return (unsigned long long)(seconds * (double)cpu_hz);
}

//-----------------------------------------------------------------------------

/// Conversion from floating point seconds to timestamp_t type
timestamp_t seconds_to_timestamp( const double& seconds, const unsigned long long& cpu_hz ) {
    timestamp_t ts;
    ts.cycle = seconds_to_cycles( seconds, cpu_hz );
    return ts;
}

//-----------------------------------------------------------------------------

/// Conversion from realtime_t type to timestamp_t type
timestamp_t realtime_to_timestamp( const realtime_t& rt, const unsigned long long& cpu_hz ) {
    timestamp_t ts;
    ts.cycle = seconds_to_cycles( rt.seconds, cpu_hz );
    return ts;
}


//-----------------------------------------------------------------------------

/// Conversion from whole nanoseconds to whole cycles
unsigned long long nanoseconds_to_cycles( const long& ns, const unsigned long long& cpu_hz ) {
    return ((unsigned long long)ns * (unsigned long long)cpu_hz)/(unsigned long long)NSECS_PER_SEC;
}

//-----------------------------------------------------------------------------

/// Conversion from whole nanoseconds to timestamp_t type
timestamp_t nanoseconds_to_timestamp( const long& ns, const unsigned long long& cpu_hz ) {
    timestamp_t ts;
    ts.cycle = nanoseconds_to_cycles( ns, cpu_hz );
    return ts;
}

//-----------------------------------------------------------------------------

/// Conversion from whole cycles to whole nanoseconds
unsigned long long cycles_to_nanoseconds( const unsigned long long& cycles, const unsigned long long& cpu_hz ) {
    return (cycles * (unsigned long long) NSECS_PER_SEC) / cpu_hz;
}

//-----------------------------------------------------------------------------

/// Conversion from timestamp_t type to whole nanoseconds
unsigned long long timestamp_to_nanoseconds( const timestamp_t& ts, const unsigned long long& cpu_hz ) {
    return cycles_to_nanoseconds( ts.cycle, cpu_hz );
}

//-----------------------------------------------------------------------------

/// Arithmetic subtraction of timespecs
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

//-----------------------------------------------------------------------------

timestamp_buffer_c::timestamp_buffer_c( void ) {
  cursor = 0;
  cpu_hz = 0;
}

//-----------------------------------------------------------------------------
timestamp_buffer_c::timestamp_buffer_c( const unsigned long long& _cpu_hz, std::string _filename ) {
  cursor = 0;
  cpu_hz = _cpu_hz;
  filename = _filename;
}

//-----------------------------------------------------------------------------

timestamp_buffer_c::~timestamp_buffer_c( void ) { }

//-----------------------------------------------------------------------------

/// initialize
error_t timestamp_buffer_c::open( void ) {
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------

/// write to disk
error_t timestamp_buffer_c::close( void ) {
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
/*
    double seconds = cycles_to_seconds( buffer[i], cpu_hz );
    sprintf( strbuffer, "%lld\t%10.9f\n", buffer[i], seconds );
    //buffer[i].seconds = cycles_to_seconds( buffer[i].cycles, cpu_hz );
    //sprintf( strbuffer, "%lld\t%10.9f\n", buffer[i].cycles, buffer[i].seconds );
    log.write( strbuffer );
  }
*/
    double seconds = timestamp_to_seconds( buffer[i], cpu_hz );
    sprintf( strbuffer, "%lld\t%10.9f\n", buffer[i].cycle, seconds );
    //buffer[i].seconds = cycles_to_seconds( buffer[i].cycles, cpu_hz );
    //sprintf( strbuffer, "%lld\t%10.9f\n", buffer[i].cycles, buffer[i].seconds );
    log.write( strbuffer );
  }

  log.close( );

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------

error_t timestamp_buffer_c::write( const timestamp_t& ts ) {
  assert( cursor < TIMESTAMP_BUFFER_SIZE - 1 );

  buffer[cursor].cycle = ts.cycle;
  cursor++;

  return ERROR_NONE;
}

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

//-----------------------------------------------------------------------------

// Note: results are too high at this point.  Revisit later
// TODO : determine more reliable calibration code
//Note: Should probably repeat this several times and average the results
// Note: querying /proc/cpuinfo is no more accurate than KHz so 1 ms ideally would
// give the same result, but there is implicit delay due to operations
unsigned long long calibrate_cycles_per_second( void ) {
    struct timespec tspec_time, tspec_period, tspec_end;
    timestamp_t ts_start, ts_end;

    printf( "Calibrating CPU speed\n" );
///*
    // ++++ Method 1.  Absolute time ++++

    tspec_period.tv_sec = 1;
    tspec_period.tv_nsec = 0;
    //tspec_period.tv_sec = 0;
    //tspec_period.tv_nsec = 1000000;

    clock_gettime( CLOCK_MONOTONIC, &tspec_time );
    tspec_end = add_timespecs( tspec_time, tspec_period );

    ts_start = generate_timestamp();
    //rdtscll( ts_start );

    clock_nanosleep( CLOCK_MONOTONIC, TIMER_ABSTIME, &tspec_end, NULL );

    ts_end = generate_timestamp();
    //rdtscll( ts_end );

    return ( ts_end.cycle - ts_start.cycle );
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

//-----------------------------------------------------------------------------

timer_c::timer_c( void ) {
  first_arming = true;
  agg_error = 0;
  last_overrun = 0;
  ts_prev_arm.cycle = 0;
}

//-------------------------------------------------------------------------

timer_c::~timer_c( void ) { }

//-------------------------------------------------------------------------
/// Blocks the timer signal if it is necessary to suppress the timer
timer_c::timer_err_e timer_c::block( void ) {
  if( sigprocmask( SIG_SETMASK, &rttimer_mask, NULL ) == -1 )
    return TIMER_ERROR_SIGPROCMASK;
  return TIMER_ERROR_NONE;
}

//-----------------------------------------------------------------------------

/// Unblocks a blocked timer signal
timer_c::timer_err_e timer_c::unblock( void ) {
  if( sigprocmask( SIG_UNBLOCK, &rttimer_mask, NULL ) == -1 )
    return TIMER_ERROR_SIGPROCMASK;
  return TIMER_ERROR_NONE;
}

//-------------------------------------------------------------------------

/// Creates a high-resolution real-time timer to monitor the controller process
timer_c::timer_err_e timer_c::create( timer_sighandler_fn sighandler, int signum ) {
  struct sigevent sevt;

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

//-------------------------------------------------------------------------

timer_c::timer_err_e timer_c::arm( const timestamp_t& ts_req, timestamp_t& ts_arm, const unsigned long long& period_nsec, const unsigned long long& cpu_speed_hz ) {
  struct itimerspec its;
  timestamp_t ts_now;

  // sanity check.  Not supporting controllers that run at 1 Hz or slower or faster than 1ns.
  //assert( CONTROLLER_HZ > 1 && CONTROLLER_HZ <= NSECS_PER_SEC );

  timestamp_t ts_err;
  ts_err.cycle = 0;

  unsigned long long ns_err = 0;

  unsigned long long period = period_nsec;

  ts_now = generate_timestamp();
  //rdtscll( ts_now.cycle );

  if( first_arming ) {
    // cannot compute err correction
    first_arming = false;
  } else {
     ts_err.cycle = ts_now.cycle - ts_prev_arm.cycle;
     ns_err = cycles_to_nanoseconds( ts_err.cycle, cpu_speed_hz );
     long long error = (long long) period - (long long) ns_err;

     agg_error += error;
     period += agg_error;
  }

  its.it_interval.tv_sec = 0;
  its.it_interval.tv_nsec = 0;
  its.it_value.tv_sec = 0;

  its.it_value.tv_nsec = (unsigned long long)(period);

  ts_arm.cycle = ts_req.cycle;
  ts_prev_arm.cycle = ts_now.cycle;

  if( timer_settime( rttimer_id, 0, &its, NULL ) == -1 ) {
    //printf( "dts: %lld, dns: %lld, nsec: %d\n", dts, dns, its.it_value.tv_nsec );
    return TIMER_ERROR_SETTIME;
  }

  return TIMER_ERROR_NONE;
}

//-----------------------------------------------------------------------------

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


