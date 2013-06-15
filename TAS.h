/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

TAS.h
-----------------------------------------------------------------------------*/

#ifndef _TAS_H_
#define _TAS_H_

//-----------------------------------------------------------------------------
/*
// C/C++ includes
#include <assert.h>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>



// POSIX includes
#include <pthread.h>

#include <errno.h>
*/

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

#include <sys/mman.h>
#include <sys/stat.h>
//#include <sys/wait.h>

//-----------------------------------------------------------------------------

typedef double Real;

#define PI 3.14159265359

#define VERBOSE 1

//-----------------------------------------------------------------------------
// IPC Channels
//-----------------------------------------------------------------------------

#define FD_COORDINATOR_TO_CONTROLLER_READ_CHANNEL   1000
#define FD_COORDINATOR_TO_CONTROLLER_WRITE_CHANNEL  1001

#define FD_CONTROLLER_TO_COORDINATOR_READ_CHANNEL   1002
#define FD_CONTROLLER_TO_COORDINATOR_WRITE_CHANNEL  1003

//-----------------------------------------------------------------------------

#define CONTROLLER_FREQUENCY_HERTZ                  1000

//-----------------------------------------------------------------------------

#define DYNAMICS_PLUGIN "/home/james/tas/build/libdynamics.so"
#define DEFAULT_CONTROLLER_PROGRAM      "controller"

//-----------------------------------------------------------------------------

#define DEFAULT_PROCESSOR    0

//-----------------------------------------------------------------------------

#define NSECS_PER_SEC 1E9
#define USECS_PER_SEC 1E6

//-----------------------------------------------------------------------------

#define DEFAULT_BUDGET_RTTIMER_NSEC	1E8
#define DEFAULT_BUDGET_RTTIMER_SEC	0

#define DEFAULT_INITDELAY_RTTIMER_NSEC	1
#define DEFAULT_INITDELAY_RTTIMER_SEC	0

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

// inline assembly for rdtsc timer
#define rdtscll(val) __asm__ __volatile__("rdtsc" : "=A" (val))

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

//-----------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------

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

#endif // _TAS_H_

