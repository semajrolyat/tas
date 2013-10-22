#include "ros/ros.h"
#include "pendulum/rpc_command.h"
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>

typedef double Real;

#define PI 3.14159265369

//-----------------------------------------------------------------------------

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


//-----------------------------------------------------------------------------
/// The position trajectory function.  A cubic
Real position_trajectory( Real t, Real tfinal, Real q0, Real qdes ) {

    if( t > tfinal )
        return qdes;
    return -2 * (qdes - q0) / (tfinal*tfinal*tfinal) * (t*t*t) + 3 * (qdes - q0) / (tfinal*tfinal) * (t*t) + q0;
}

//-----------------------------------------------------------------------------
/// The velocity trajectory function.  The first derivative of the cubic
Real velocity_trajectory( Real t, Real tfinal, Real q0, Real qdes ) {

    if( t > tfinal )
        return 0.0;
    return -6 * (qdes - q0) / (tfinal*tfinal*tfinal) * (t*t) + 6 * (qdes - q0) / (tfinal*tfinal) * (t);
}

//-----------------------------------------------------------------------------
/// Control function for Standalone Controller
Real control( Real time, Real position, Real velocity ) {
    //const Real Kp = 1.0;
    //const Real Kv = 0.1;

    const Real Kp = 100.0;
    const Real Kv = 10.0;

    Real measured_position = position;
    Real measured_velocity = velocity;

    //Real desired_position = position_trajectory( time, 0.5, 0.0, PI );
    //Real desired_velocity = velocity_trajectory( time, 0.5, 0.0, PI );

    Real desired_position = position_trajectory( time, 10.0, 0.0, PI );
    Real desired_velocity = velocity_trajectory( time, 10.0, 0.0, PI );

    Real torque = Kp * ( desired_position - measured_position ) + Kv * ( desired_velocity - measured_velocity );

    return torque;
}

//-----------------------------------------------------------------------------

static int pid;
std::stringstream ss_proc_file("");

int unsigned long last_time = 0;

//-----------------------------------------------------------------------------
bool control(pendulum::rpc_command::Request &req, pendulum::rpc_command::Response &res) {
  res.torque = control( req.time, req.position, req.velocity );

  proc_stat_t stats;
  read_proc_stat( ss_proc_file.str().c_str(), &stats );
  unsigned long systime = stats.utime;

  if( systime > last_time )
    ROS_INFO(" %ld, %10.9f, %10.9f, %10.9f, %10.9f", systime, req.time, req.position, req.velocity, res.torque);

  last_time = systime;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pendulum_controller");
  //ros::init(argc, argv, "pendulum");
  ros::NodeHandle n("pendulum");

  pid = getpid( );
  ss_proc_file << "/proc" << "/" << pid << "/stat";

  ros::ServiceServer service = n.advertiseService("pendulum_standup_command", control);
  ROS_INFO("Ready to standup pendulum.");
  ROS_INFO(" systime, simtime, position, velocity, torque");

  ros::spin();

  return 0;
}
