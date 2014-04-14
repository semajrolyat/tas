#include <vector>
#include <sys/types.h>
#include <sys/mman.h>
#include <stdio.h>

#include "cpu.h"
#include "timer.h"
#include "os.h"
#include "log.h"
#include "types.h"
#include "notification.h"
#include "message.h"
#include "processor.h"
#include "timesink.h"
#include "osthread.h"
#include "dynamics.h"
#include "channels.h"

//-----------------------------------------------------------------------------
static pid_t         coordinator_pid;
static int           coordinator_priority;
static cpu_speed_t   cpu_speed;
static cpu_id_t      cpu;
static int           CLIENT_MAX_PRIORITY;
static int           CLIENT_MIN_PRIORITY;
static int           wakeup_priority;
static int           timer_priority;

//-----------------------------------------------------------------------------
timer_c              timer;
//-----------------------------------------------------------------------------
char errstr[ 256 ];

//-----------------------------------------------------------------------------
#define LOG_CAPACITY 4096
log_c info;
//-----------------------------------------------------------------------------

std::vector<int>        subscribed_fds;
fd_set                  pending_fds;

notification_t note;
// note.ts
// note.t
// note.source = {TIMER,WAKEUP,CLIENT}
// note.type = {IDLE,OPEN,CLOSE,READ,WRITE}

client_message_t msg;
client_message_buffer_c msgbuffer;

//-----------------------------------------------------------------------------
// Pthreads
pthread_t wakeup_thread;

//-----------------------------------------------------------------------------
// Threads
boost::shared_ptr<processor_c> processor;

boost::shared_ptr<dynamics_c> dynamics;

boost::shared_ptr<timesink_c> prey;
boost::shared_ptr<osthread_c> prey_controller;

boost::shared_ptr<timesink_c> pred;
boost::shared_ptr<osthread_c> pred_controller;
boost::shared_ptr<osthread_c> pred_planner;

//-----------------------------------------------------------------------------
void select( void ) {
  __select( subscribed_fds, pending_fds );
}

//-----------------------------------------------------------------------------
void read_notifications( void ) {
  notification_t note;
  int fd;
  std::string err;

  if(__isset(FD_TIMER_TO_COORDINATOR_READ_CHANNEL, pending_fds)) {
    fd = FD_TIMER_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      err = "(coordinator.cpp) read_messages() failed calling __read(FD_TIMER_TO_COORDINATOR_READ_CHANNEL,...)";
      // TODO: write to error log
    } 
  } else if(__isset(FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL, pending_fds)) {
    fd = FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      err = "(coordinator.cpp) read_messages() failed calling __read(FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL,...)";
      // TODO: write to error log
    } 
  } else if(__isset(FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL, pending_fds)) {
    fd = FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      err = "(coordinator.cpp) read_messages() failed calling __read(FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL,...)";
      // TODO: write to error log
    } 
  } else if(__isset(FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL, pending_fds)) {
    fd = FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      err = "(coordinator.cpp) read_messages() failed calling __read(FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL,...)";
      // TODO: write to error log
    } 
  } else if(__isset(FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL, pending_fds)) {
    fd = FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      err = "(coordinator.cpp) read_messages() failed calling __read(FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL,...)";
      // TODO: write to error log
    } 
  }
}

//-----------------------------------------------------------------------------
void timer_sighandler( int signum, siginfo_t *si, void *data ) {

  notification_t note;
  static int fd = FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL;
  std::string err;

  note.ts = generate_timestamp();
  note.source = notification_t::TIMER;

  if( __write( fd, &note, sizeof(notification_t) ) == -1 ) {
    err = "(coordinator.cpp) timer_sighandler(...) failed making system call write(...)";
    //error_log.write( error_string_bad_write( err, errno ) );
    // TODO : determine if there is a need to recover
  }
}

//-----------------------------------------------------------------------------
void* wakeup( void* ) {
    char buf;
    timespec ts_sleep, ts_rem;
    ts_sleep.tv_sec = 0;
    ts_sleep.tv_nsec = 100;

    while( 1 ) {
        // REMARKED FOR NOW TO PREVENT LOCKUP UNTIL THIS GETS CLOSER VETTING AND INSTEAD SLEEP
        /*
        buf = 0;

        if( controller_thread.blocked ) {
            //clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, time, NULL);
            //nanosleep( &ts_sleep, &ts_rem );  // DONT USE.  USE clock_nanosleep(...)
            continue;
        }

        // Note: may need mutex protection!
        controller_thread.blocked = true;

        if( write( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL, &buf, 1 ) == -1) {
            std::string err_msg = "(coordinator.cpp) wakeup() failed making system call write(...)";
            error_log.write( error_string_bad_write( err_msg, errno ) );
            // TODO : determine if there is a need to bomb or recover
        }
        */
        clock_nanosleep( CLOCK_MONOTONIC, 0, &ts_sleep, &ts_rem );
    }
    return NULL;
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
//-----------------------------------------------------------------------------
bool init_timer_pipe( void ) {
  return init_pipe( FD_TIMER_TO_COORDINATOR_READ_CHANNEL, FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_timer_pipe( void ) {
  __close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_wakeup_pipe( void ) {
  return init_pipe( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL, FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_wakeup_pipe( void ) {
  __close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_coordinator_to_preycontroller_pipe( void ) {
  return init_pipe( FD_COORDINATOR_TO_PREYCONTROLLER_READ_CHANNEL, FD_COORDINATOR_TO_PREYCONTROLLER_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_coordinator_to_preycontroller_pipe( void ) {
  __close( FD_COORDINATOR_TO_PREYCONTROLLER_READ_CHANNEL );
  __close( FD_COORDINATOR_TO_PREYCONTROLLER_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
bool init_preycontroller_to_coordinator_pipe( void ) {
  return init_pipe( FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL, FD_PREYCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_preycontroller_to_coordinator_pipe( void ) {
  __close( FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_PREYCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_coordinator_to_predplanner_pipe( void ) {
  return init_pipe( FD_COORDINATOR_TO_PREDPLANNER_READ_CHANNEL, FD_COORDINATOR_TO_PREDPLANNER_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_coordinator_to_predplanner_pipe( void ) {
  __close( FD_COORDINATOR_TO_PREDPLANNER_READ_CHANNEL );
  __close( FD_COORDINATOR_TO_PREDPLANNER_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
bool init_predplanner_to_coordinator_pipe( void ) {
  return init_pipe( FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL, FD_PREDPLANNER_TO_COORDINATOR_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_predplanner_to_coordinator_pipe( void ) {
  __close( FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_PREDPLANNER_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_coordinator_to_predcontroller_pipe( void ) {
  return init_pipe( FD_COORDINATOR_TO_PREDCONTROLLER_READ_CHANNEL, FD_COORDINATOR_TO_PREDCONTROLLER_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_coordinator_to_predcontroller_pipe( void ) {
  __close( FD_COORDINATOR_TO_PREDCONTROLLER_READ_CHANNEL );
  __close( FD_COORDINATOR_TO_PREDCONTROLLER_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
bool init_predcontroller_to_coordinator_pipe( void ) {
  return init_pipe( FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL, FD_PREDCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_predcontroller_to_coordinator_pipe( void ) {
  __close( FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_PREDCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_pipes( void ) {
  if( !init_timer_pipe() ) {
    return false;
  }

  if( !init_wakeup_pipe() ) {
    close_timer_pipe();
    return false;
  }

  if( !init_coordinator_to_preycontroller_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    return false;
  }
  if( !init_preycontroller_to_coordinator_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    close_coordinator_to_preycontroller_pipe();
    return false;
  }

  if( !init_coordinator_to_predplanner_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    close_coordinator_to_preycontroller_pipe();
    close_preycontroller_to_coordinator_pipe();
    return false;
  }
  if( !init_predplanner_to_coordinator_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    close_coordinator_to_preycontroller_pipe();
    close_preycontroller_to_coordinator_pipe();
    close_coordinator_to_predplanner_pipe();
    return false;
  }

  if( !init_coordinator_to_predcontroller_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    close_coordinator_to_preycontroller_pipe();
    close_preycontroller_to_coordinator_pipe();
    close_coordinator_to_predplanner_pipe();
    close_predplanner_to_coordinator_pipe();
    return false;
  }
  if( !init_predcontroller_to_coordinator_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    close_coordinator_to_preycontroller_pipe();
    close_preycontroller_to_coordinator_pipe();
    close_coordinator_to_predplanner_pipe();
    close_predplanner_to_coordinator_pipe();
    close_coordinator_to_predcontroller_pipe();
    return false;
  }

  subscribed_fds.push_back( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
  subscribed_fds.push_back( FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL );
  subscribed_fds.push_back( FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL );
  subscribed_fds.push_back( FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL );
  subscribed_fds.push_back( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );

  return true;
}

//-----------------------------------------------------------------------------
void close_pipes( void ) {
  close_timer_pipe();
  close_wakeup_pipe();
  close_coordinator_to_preycontroller_pipe();
  close_preycontroller_to_coordinator_pipe();
  close_coordinator_to_predplanner_pipe();
  close_predplanner_to_coordinator_pipe();
  close_coordinator_to_predcontroller_pipe();
  close_predcontroller_to_coordinator_pipe();
}

//-----------------------------------------------------------------------------
void init( int argc, char* argv[] ) {
  // * set variables from constants *
  cpu = DEFAULT_CPU;

  // * open error logs *
  std::string log_name = "info.log";

  info = log_c( log_name );
  log_c::error_e log_err;
  log_err = info.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling log_c::allocate(...).\nExiting\n" ); 
    printf( "%s", errstr );
    exit( 1 );
  }

  // * get the process identifier *
  coordinator_pid = getpid( );
  printf( "coordinator pid: %d\n", coordinator_pid );

  // * bind the process to a single cpu *
  if( cpu_c::bind( coordinator_pid, cpu ) != cpu_c::ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling cpu_c::bind(coordinator_pid,DEFAULT_CPU).\nExiting\n" );
    //error_log.write( errstr );
    printf( "%s", errstr );

    //error_log.close( );
    exit( 1 );
  }

  // * set the process to be scheduled with realtime policy and max priority *
  if( scheduler_c::set_realtime_policy( coordinator_pid, coordinator_priority ) != scheduler_c::ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling schedule_set_realtime_max(coordinator_pid,coordinator_priority).\nExiting\n" );
    //error_log.write( errstr );
    printf( "%s", errstr );

    //error_log.close( );
    exit( 1 );
  }
  printf( "coordinator process priority: %d\n", coordinator_priority );

  // * determine if the OS supports high resolution timers *
  struct timespec res;
  clock_getres( CLOCK_MONOTONIC, &res );
  double clock_res = timespec_to_real( res );
  printf( "Clock resolution (secs): %10.9f\n", clock_res );

  if( res.tv_sec != 0 && res.tv_nsec != 1 ) {
    sprintf( errstr, "(coordinator.cpp) init() failed.  The host operating system does not support high resolution timers.  Consult your system documentation on enabling this feature.\nExiting\n" );
    //error_log.write( errstr );
    printf( "%s", errstr );

    //error_log.close( );
    exit( 1 );
  }

  // * get the cpu speed *
  if( cpu_c::get_speed( cpu_speed, cpu ) != cpu_c::ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling cpu_c::get_frequency(cpu_speed,cpu)\nExiting\n" );
    //error_log.write( errstr );
    printf( "%s", errstr );

    //error_log.close( );
    exit( 1 );
  }

  // * initialize pipes *
  if( !init_pipes() ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling init_pipes()\nExiting\n" );
    //error_log.write( errstr );
    printf( "%s", errstr );

    //error_log.close( );
    exit( 1 );
  }

  // * initialize shared memory *
  msgbuffer = client_message_buffer_c( CLIENT_MESSAGE_BUFFER_NAME, CLIENT_MESSAGE_BUFFER_MUTEX_NAME, true );
  if( msgbuffer.open( ) != client_message_buffer_c::ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling actuator_msg_buffer_c.open(...,true)\n" );
    //error_log.write( errstr );
    printf( "%s", errstr );

    close_pipes( );
    //error_log.close( );
    exit( 1 );
  }

  // * initialize clients *
/*
  CLIENT_MAX_PRIORITY = coordinator_priority - 1;
  CLIENT_MIN_PRIORITY = coordinator_priority - 3;
  int client_priority_step = CLIENT_MAX_PRIORITY - CLIENT_MIN_PRIORITY;

  processor = boost::shared_ptr<processor_c>( new processor_c() );
  processor->name = "processor";

  dynamics = boost::shared_ptr<dynamics_c>( new dynamics_c() );
  dynamics->name = "dynamics";

  prey = boost::shared_ptr<timesink_c>( new timesink_c( scheduler_c::PROGRESS) );
  prey->name = "prey_timesink";
  prey_controller = boost::shared_ptr<osthread_c>( new osthread_c(&select, &read_notifications) );
  prey_controller->name = "prey_controller";
  prey_controller->_max_priority = CLIENT_MAX_PRIORITY;
  prey_controller->_min_priority = CLIENT_MIN_PRIORITY;
  prey_controller->_priority_step = client_priority_step;

  pred = boost::shared_ptr<timesink_c>( new timesink_c(scheduler_c::PRIORITY) );
  pred->name = "pred_timesink";
  pred_controller = boost::shared_ptr<osthread_c>( new osthread_c(&select, &read_notifications) );
  pred_controller->name = "pred_controller";
  pred_controller->_max_priority = CLIENT_MAX_PRIORITY;
  pred_controller->_min_priority = CLIENT_MIN_PRIORITY;
  pred_controller->_priority_step = client_priority_step;
  pred_planner = boost::shared_ptr<osthread_c>( new osthread_c(&select, &read_notifications) );
  pred_planner->name = "pred_planner";
  pred_planner->_max_priority = CLIENT_MAX_PRIORITY;
  pred_planner->_min_priority = CLIENT_MIN_PRIORITY;
  pred_planner->_priority_step = client_priority_step;

  processor->run_queue.push_back( dynamics );
  processor->run_queue.push_back( prey );
  processor->run_queue.push_back( pred );
  std::make_heap(processor->run_queue.begin(),processor->run_queue.end(),compare_thread_p_t());

  prey->run_queue.push_back( prey_controller );
  std::make_heap(prey->run_queue.begin(),prey->run_queue.end(),compare_thread_p_t());

  pred->run_queue.push_back( pred_planner );
  pred->run_queue.push_back( pred_controller );
  std::make_heap(pred->run_queue.begin(),pred->run_queue.end(),compare_thread_p_t());
*/
  // * initialize block detection, i.e. wakeup * 
  wakeup_priority = coordinator_priority - 2;

  if( scheduler_c::create( wakeup_thread, wakeup_priority, wakeup ) != scheduler_c::ERROR_NONE ) {
     msgbuffer.close( );
     close_pipes( );
     //error_log.close( );
     printf( "(coordinator.cpp) init() failed calling wakeup_thread.create(...,wakeup)\nExiting\n" );
     exit( 1 );
  }

  // * initialize timer *
  if( timer.create( timer_sighandler, RTTIMER_SIGNAL ) != timer_c::ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling timer.create(timer_sighandler,RTTIMER_SIGNAL)\nExiting\n" );
    //error_log.write( errstr );
    printf( "%s", errstr );

    msgbuffer.close( );
    close_pipes( );
    //error_log.close( );
    exit( 1 );
  }


  // * initialize other resources *

  // lock into memory to prevent pagefaults.  do last before main loop
  mlockall( MCL_CURRENT );
} 

//-----------------------------------------------------------------------------
void shutdown( void ) {
  // unlock memory.  do before any other shutdown operations
  munlockall();

  printf( "shutting down\n" );

  // send notifications to clients

  msgbuffer.close();
  close_pipes();
}

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] ) {
  // Due to realtime scheduling, et al, must have root access to run.
  if( geteuid() != 0 ) {
    sprintf( errstr, "This program requires root access.  Re-run with sudo.\nExiting\n" );
    printf( "%s", errstr );
    exit( 1 );
  }

  init( argc, argv );

  thread_p processor_thread = boost::dynamic_pointer_cast<thread_c>(processor);
  while( true ) {
    scheduler_c::step_system( processor_thread, processor->run_queue, processor->block_queue, &info );
    //sleep( 1 ); // temporary sleep for testing.  Induces block, otherwise realtime schedule max will suck up the entire processor if there is no blocking mechanism
  }

  shutdown();
  return 0;
}

