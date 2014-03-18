#include "../cpu.h"
#include "../timer.h"
#include "../os.h"
#include "../notification.h"
#include "../scheduler.h"

#include <string>
#include <stdio.h>
#include <stdlib.h>

//-----------------------------------------------------------------------------

#define FD_TIMER_READ_CHANNEL   1001
#define FD_TIMER_WRITE_CHANNEL  1002

//-----------------------------------------------------------------------------
static pid_t         pid;
static int           priority;
static cpu_speed_t   cpu_speed;
static cpu_id_t      cpu;
static int           timer_priority;

timer_c timer;
char errstr[ 256 ];

//-----------------------------------------------------------------------------
void timer_sighandler( int signum, siginfo_t *si, void *data ) {

  notification_t note;
  static int fd = FD_TIMER_WRITE_CHANNEL;
  std::string err;

  note.ts = generate_timestamp();
  note.source = notification_t::TIMER;

  if( __write( fd, &note, sizeof(notification_t) ) == -1 ) {
    err = "(test.cpp) timer_sighandler(...) failed making system call write(...)";
    //error_log.write( error_string_bad_write( err, errno ) );
    // TODO : determine if there is a need to recover
  }
}

//-----------------------------------------------------------------------------
#include <unistd.h>
#include <fcntl.h>
bool init_pipe( int read_fd, int write_fd, bool write_blocking = false ) {
  int flags;
  int fd[2];

  if( pipe( fd ) != 0 ) {
    return false;
  }
  flags = fcntl( fd[0], F_GETFL, 0 );
  fcntl( fd[0], F_SETFL, flags );
  flags = fcntl( fd[1], F_GETFL, 0 );
  if( write_blocking )
    fcntl( fd[1], F_SETFL, flags );
  else
    fcntl( fd[1], F_SETFL, flags | O_NONBLOCK );

  if( dup2( fd[0], read_fd ) == -1 ) {
    __close( fd[0] );
    __close( fd[1] );
    return false;
  }
  if( dup2( fd[1], write_fd ) == -1 ) {
    __close( read_fd );
    __close( fd[1] );
    return false;
  }
  return true;

}

//-----------------------------------------------------------------------------
bool init_timer_pipe( void ) {
  return init_pipe( FD_TIMER_READ_CHANNEL, FD_TIMER_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
void close_timer_pipe( void ) {
  __close( FD_TIMER_READ_CHANNEL );
  __close( FD_TIMER_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
bool init_pipes( void ) {
  if( !init_timer_pipe() ) {
    return false;
  }
  return true;
}

//-----------------------------------------------------------------------------
void close_pipes( void ) {
  close_timer_pipe();
}

//-----------------------------------------------------------------------------
void init( void ) {
  // * set variables from constants *
  cpu = DEFAULT_CPU;

  // * open error logs *

  // * get the process identifier *
  pid = getpid( );
  printf( "pid: %d\n", pid );

  // * bind the process to a single cpu *
  if( cpu_c::bind( pid, cpu ) != cpu_c::ERROR_NONE ) {
    sprintf( errstr, "(test_timer.cpp) init() failed calling cpu_c::_bind(pid,DEFAULT_CPU).\nExiting\n" );
    //error_log.write( errstr );
    printf( "%s", errstr );

    //error_log.close( );
    exit( 1 );
  }

  // * set the process to be scheduled with realtime policy and max priority *
  if( scheduler_c::set_realtime_policy( pid, priority ) != scheduler_c::ERROR_NONE ) {
    sprintf( errstr, "(test_timer.cpp) init() failed calling schedule_set_realtime_max(pid,priority).\nExiting\n" );
    //error_log.write( errstr );
    printf( "%s", errstr );

    //error_log.close( );
    exit( 1 );
  }
  printf( "process priority: %d\n", priority );

  // * determine if the OS supports high resolution timers *
  struct timespec res;
  clock_getres( CLOCK_MONOTONIC, &res );
  double clock_res = timespec_to_real( res );
  printf( "Clock resolution (secs): %10.9f\n", clock_res );

  if( res.tv_sec != 0 && res.tv_nsec != 1 ) {
    sprintf( errstr, "(test_timer.cpp) init() failed.  The host operating system does not support high resolution timers.  Consult your system documentation on enabling this feature.\nExiting\n" );
    //error_log.write( errstr );
    printf( "%s", errstr );

    //error_log.close( );
    exit( 1 );
  }

  // * get the cpu speed *
  if( cpu_c::get_speed( cpu_speed, cpu ) != cpu_c::ERROR_NONE ) {
    sprintf( errstr, "(test_timer.cpp) init() failed calling cpu_c::get_frequency(cpu_speed,cpu)\nExiting\n" );
    //error_log.write( errstr );
    printf( "%s", errstr );

    //error_log.close( );
    exit( 1 );
  }

  // * initialize pipes *
  if( !init_pipes() ) {
    sprintf( errstr, "(test_timer.cpp) init() failed calling init_pipes()\nExiting\n" );
    //error_log.write( errstr );
    printf( "%s", errstr );

    //error_log.close( );
    exit( 1 );
  }

  // * initialize timer *
  if( timer.create( timer_sighandler, RTTIMER_SIGNAL ) != timer_c::ERROR_NONE ) {
    sprintf( errstr, "(test_timer.cpp) init() failed calling timer.create(timer_sighandler,RTTIMER_SIGNAL)\nExiting\n" );
    //error_log.write( errstr );
    printf( "%s", errstr );

    close_pipes( );
    //error_log.close( );
    exit( 1 );
  }

}

//-----------------------------------------------------------------------------
int main( void ) {
  // Due to realtime scheduling, et al, must have root access to run.
  if( geteuid() != 0 ) {
    sprintf( errstr, "This program requires root access.  Re-run with sudo.\nExiting\n" );
    printf( "%s", errstr );
    exit( 1 );
  }

  printf( "sizeof(long): %d\n", sizeof( long ) );
  printf( "sizeof(long int): %d\n", sizeof( long int ) );

  init( );
/*
  std::string err;

  std::vector<int> fds;
  fd_set set;
  fds.push_back( FD_TIMER_READ_CHANNEL );

  timestamp_t arm_ts;

  unsigned long long PERIOD_NSEC = (unsigned long long)2E9;

  //timer.arm( generate_timestamp(), arm_ts, PERIOD, cpu_speed );
  //timer.unblock();

  //timer_c::type_e timer_type = timer_c::PERIODIC;
  timer_c::type_e timer_type = timer_c::ONESHOT;

  if( timer_type == timer_c::PERIODIC ) {
    if( timer.arm( timer_type, generate_timestamp(), arm_ts, PERIOD_NSEC, cpu_speed ) != timer_c::ERROR_NONE ) {
      err = "(test_timer.cpp) main() failed calling timer.arm(...)";
      printf( "error: %s\n", err.c_str() );
    }
  }

  printf( "starting main loop\n" );

  while( true ) {
    if( timer_type == timer_c::ONESHOT ) {
      if( timer.arm( timer_type, generate_timestamp(), arm_ts, PERIOD_NSEC, cpu_speed ) != timer_c::ERROR_NONE ) {
        err = "(test_timer.cpp) main() failed calling timer.arm(...)";
        printf( "error: %s\n", err.c_str() );
      }
    }

    __select( fds, set );
    if( __isset( FD_TIMER_READ_CHANNEL, set ) ) {
      int fd = FD_TIMER_READ_CHANNEL;
      notification_t note;
      if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
        err = "(test_timer.cpp) main() failed calling __read(FD_TIMER_READ_CHANNEL,...)";
        printf( "error: %s\n", err.c_str() );
      }
      printf( "received timer notification: ts[%llu]\n", note.ts );
    }
    sleep(1);
  }
*/
  std::string err;
  timestamp_t arm_ts;
  unsigned long long PERIOD_NSEC = 2E9;

  //timer_c::type_e timer_type = timer_c::PERIODIC;
  timer_c::type_e timer_type;

  // set up ipc
  std::vector<int> fds;
  fd_set set;
  fds.push_back( FD_TIMER_READ_CHANNEL );

  // * Test One Shot Timer *
  timer_type = timer_c::ONESHOT;

  printf( "starting one shot timer main loop\n" );

  for( unsigned i = 0; i < 10; i++ ) {
    // a one shot timer is armed each time through the main loop
    if( timer.arm( timer_type, PERIOD_NSEC ) != timer_c::ERROR_NONE ) {
      err = "(test_timer.cpp) main() failed calling timer.arm(...)";
      printf( "error: %s\n", err.c_str() );
    }

    // block on select
    __select( fds, set );
    // process the message
    if( __isset( FD_TIMER_READ_CHANNEL, set ) ) {
      int fd = FD_TIMER_READ_CHANNEL;
      notification_t note;
      if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
        err = "(test_timer.cpp) main() failed calling __read(FD_TIMER_READ_CHANNEL,...)";
        printf( "error: %s\n", err.c_str() );
      }
      printf( "received timer notification: i[%d] ts[%llu]\n", i, note.ts );
    }
  }
  printf( "one-shot timer main loop completed\n" );

  // * Test Periodic Timer *
  timer_type = timer_c::PERIODIC;

  // a periodic timer is armed only once
  if( timer.arm( timer_type, PERIOD_NSEC ) != timer_c::ERROR_NONE ) {
    err = "(test_timer.cpp) main() failed calling timer.arm(...)";
    printf( "error: %s\n", err.c_str() );
  }

  printf( "starting periodic timer main loop\n" );

  for( unsigned i = 0; i < 10; i++ ) {
    // block on select
    __select( fds, set );
    // process the message
    if( __isset( FD_TIMER_READ_CHANNEL, set ) ) {
      int fd = FD_TIMER_READ_CHANNEL;
      notification_t note;
      if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
        err = "(test_timer.cpp) main() failed calling __read(FD_TIMER_READ_CHANNEL,...)";
        printf( "error: %s\n", err.c_str() );
      }
      printf( "received timer notification: i[%d] ts[%llu]\n", i, note.ts );
    }
  }
  printf( "periodic timer main loop completed\n" );



  return 0;
}
