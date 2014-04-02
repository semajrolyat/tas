#include <sstream>
#include <vector>
#include <sys/types.h>
#include <sys/mman.h>
#include <stdio.h>

#include "../cpu.h"
#include "../timer.h"
#include "../os.h"
#include "../types.h"
#include "../notification.h"
#include "../message.h"
#include "../processor.h"
#include "../timesink.h"
#include "../osthread.h"
#include "../dynamics.h"

#include "../channels.h"

//-----------------------------------------------------------------------------
static pid_t         coordinator_pid;
static int           coordinator_os_priority;
static cpu_speed_t   cpu_speed;
static cpu_id_t      cpu;
static int           CLIENT_OS_MAX_PRIORITY;
static int           CLIENT_OS_MIN_PRIORITY;
static int           wakeup_os_priority;
static int           timer_os_priority;

static pid_t         worker_pid;

//-----------------------------------------------------------------------------
timer_c              timer;
unsigned             caught_timer_events;
unsigned             actual_timer_events;
#define MAX_TIMER_EVENTS 50
#define TIMER_PERIOD_NSECS 100000
//-----------------------------------------------------------------------------

bool quit;
//-----------------------------------------------------------------------------
//char errstr[ 256 ];
char spstr[512];

//-----------------------------------------------------------------------------
#define LOG_CAPACITY 4096
log_c log;
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
void log_msg( const char* msg ) {
  if( log.open() ) log.write( msg );
  //printf( "%s", msg );
}

//-----------------------------------------------------------------------------
void select( void ) {
  __select( subscribed_fds, pending_fds );
}

//-----------------------------------------------------------------------------
void read_notifications( void ) {
  notification_t note;
  int fd;

  if(__isset(FD_TIMER_TO_COORDINATOR_READ_CHANNEL, pending_fds)) {
    fd = FD_TIMER_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      sprintf( spstr, "(coordinator.cpp) read_messages() failed calling __read(FD_TIMER_TO_COORDINATOR_READ_CHANNEL,...)" );
      log_msg( spstr );
    }
    sprintf( spstr, "(coordinator.cpp::read_notifications) Timer fired. caught_timer_events:%d\n", caught_timer_events );
    log_msg( spstr );

    if( caught_timer_events >= MAX_TIMER_EVENTS )
      quit = true;
  } else if(__isset(FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL, pending_fds)) {
    fd = FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      sprintf( spstr, "(coordinator.cpp) read_messages() failed calling __read(FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL,...)" );
      log_msg( spstr );
    }

    sprintf( spstr, "(coordinator.cpp::read_notifications) Received prey_controller notification\n" );
    log_msg( spstr );

    prey_controller->message_queue.push( note );
  } else if(__isset(FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL, pending_fds)) {
    fd = FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      sprintf( spstr, "(coordinator.cpp) read_messages() failed calling __read(FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL,...)" );
      log_msg( spstr );
    } 

    sprintf( spstr, "(coordinator.cpp::read_notifications) Received pred_planner notification\n" );
    log_msg( spstr );

    pred_planner->message_queue.push( note );
  } else if(__isset(FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL, pending_fds)) {
    fd = FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      sprintf( spstr, "(coordinator.cpp) read_messages() failed calling __read(FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL,...)" );
      log_msg( spstr );
    } 

    sprintf( spstr, "(coordinator.cpp::read_notifications) Received pred_controller notification\n" );
    log_msg( spstr );

    pred_controller->message_queue.push( note );
  } else if(__isset(FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL, pending_fds)) {
    fd = FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      sprintf( spstr, "(coordinator.cpp) read_messages() failed calling __read(FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL,...)" );
      log_msg( spstr );
    }
 
    sprintf( spstr, "(coordinator.cpp::read_notifications) Block detected\n" );
    log_msg( spstr );
  }
}

//-----------------------------------------------------------------------------
void timer_sighandler( int signum, siginfo_t *si, void *data ) {

  notification_t note;
  static int fd = FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL;
  std::string err, eno;

  actual_timer_events++;

  note.ts = generate_timestamp();
  note.source = notification_t::TIMER;
  
  if( __write( fd, &note, sizeof(notification_t) ) == -1 ) {
    if( errno == EPIPE )
      eno = " errno: EPIPE";
    else if( errno == EAGAIN || errno == EWOULDBLOCK )
      eno = " errno: EWOULDBLOCK";
    else if( errno == EBADF )
      eno = " errno: EBADF";
    else if( errno == EDESTADDRREQ )
      eno = " errno: EDESTADDRREQ";
    else if( errno == EDQUOT )
      eno = " errno: EDQUOT";
    else if( errno == EFAULT )
      eno = " errno: EFAULT";
    else if( errno == EFBIG )
      eno = " errno: EFBIG";
    else if( errno == EINTR )
      eno = " errno: EINTR";
    else if( errno == EINVAL )
      eno = " errno: EINVAL";
    else if( errno == EIO )
      eno = " errno: EIO";
    else if( errno == ENOSPC )
      eno = " errno: ENOSPC";

    err = "(coordinator.cpp) timer_sighandler(...) failed making system call write(...)" + eno;
    //sprintf( spstr, "(coordinator.cpp) timer_sighandler(...) failed making system call write(...) %s\n", eno.c_str() );
    sprintf( spstr, "%s\n", err.c_str() );
    log_msg( spstr );
    //error_log.write( error_string_bad_write( err, errno ) );
    // TODO : determine if there is a need to recover
  }
}

//-----------------------------------------------------------------------------
void* wakeup( void* ) {

  notification_t note;
  static int fd = FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL;
  timespec ts_sleep, ts_rem;

  ts_sleep.tv_sec = 0;
  ts_sleep.tv_nsec = 100;

  note.source = notification_t::WAKEUP;

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
        */
    note.ts = generate_timestamp();
    if( write( fd, &note, sizeof(notification_t) ) == -1 ) {
      sprintf( spstr, "(coordinator.cpp) wakeup() failed making system call write(...)\n" );
      log_msg( spstr );
      //error_log.write( error_string_bad_write( err_msg, errno ) );
      // TODO : determine if there is a need to bomb or recover
    }
    //*/
    //clock_nanosleep( CLOCK_MONOTONIC, 0, &ts_sleep, &ts_rem );
  }
  return NULL;
}


//-----------------------------------------------------------------------------
#include <unistd.h>
#include <fcntl.h>
bool init_pipe( const int& read_fd, const int& write_fd, bool write_blocking = false ) {
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
  quit = false;
  actual_timer_events = 0;
  caught_timer_events = 0;

  // * set variables from constants *
  cpu = DEFAULT_CPU;

  // * open log *
  log = log_c( "info.log", true );
  log_c::error_e log_err = log.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling log_c::allocate(...).\nExiting\n" ); 
    log_msg( spstr );
    exit( 1 );
  }

  // * get the process identifier *
  coordinator_pid = getpid( );
  
  sprintf( spstr, "coordinator pid: %d\n", coordinator_pid );
  log_msg( spstr );

  // * bind the process to a single cpu *
  if( cpu_c::bind( coordinator_pid, cpu ) != cpu_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling cpu_c::_bind(coordinator_pid,DEFAULT_CPU).\nExiting\n" );
    log_msg( spstr );
    exit( 1 );
  }

  // * set the process to be scheduled with realtime policy and max priority *
  if( scheduler_c::set_realtime_policy( coordinator_pid, coordinator_os_priority ) != scheduler_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling schedule_set_realtime_max(coordinator_pid,coordinator_priority).\nExiting\n" );
    log_msg( spstr );
    exit( 1 );
  }
  sprintf( spstr, "coordinator os priority: %d\n", coordinator_os_priority );
  log_msg( spstr );

  // * determine if the OS supports high resolution timers *
  struct timespec res;
  clock_getres( CLOCK_MONOTONIC, &res );
  double clock_res = timespec_to_real( res );

  sprintf( spstr, "clock resolution (secs): %10.9f\n", clock_res );
  log_msg( spstr );
 
  if( res.tv_sec != 0 && res.tv_nsec != 1 ) {
    sprintf( spstr, "(coordinator.cpp) init() failed.  The host operating system does not support high resolution timers.  Consult your system documentation on enabling this feature.\nExiting\n" );
    log_msg( spstr );
    exit( 1 );
  }

  // * get the cpu speed *
  if( cpu_c::get_speed( cpu_speed, cpu ) != cpu_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling cpu_c::get_frequency(cpu_speed,cpu)\nExiting\n" );
    log_msg( spstr );
    exit( 1 );
  }
  sprintf( spstr, "cpu speed(hz): %llu\n", cpu_speed );
  log_msg( spstr );

  // * initialize pipes *
  if( !init_pipes() ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling init_pipes()\nExiting\n" );
    log_msg( spstr );
    exit( 1 );
  }

  // * initialize shared memory *
  msgbuffer = client_message_buffer_c( CLIENT_MESSAGE_BUFFER_NAME, CLIENT_MESSAGE_BUFFER_MUTEX_NAME, true );
  if( msgbuffer.open( ) != client_message_buffer_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling actuator_msg_buffer_c.open(...,true)\nExiting\n" );
    log_msg( spstr );
    close_pipes( );
    exit( 1 );
  }

  // * initialize block detection, i.e. wakeup * 
  wakeup_os_priority = coordinator_os_priority - 2;

  if( scheduler_c::create( wakeup_thread, wakeup_os_priority, wakeup ) != scheduler_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling wakeup_thread.create(...,wakeup)\nExiting\n" );
    log_msg( spstr );
    msgbuffer.close( );
    close_pipes( );
    exit( 1 );
  }

  if( scheduler_c::get_priority( wakeup_thread, wakeup_os_priority ) != scheduler_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling get_priority(wakeup_thread,...)\nExiting\n" );
    log_msg( spstr );
    msgbuffer.close( );
    close_pipes( );
    // kill the wakeup thread?
    exit(1);
  }
  sprintf( spstr, "wakeup thread created... priority:%d\n", wakeup_os_priority );
  log_msg( spstr );

  sprintf( spstr, "initializing clients\n" );
  log_msg( spstr );

  // * initialize clients *
  scheduler_c::error_e schedulererr;

  const char* controller_floor = "500";
  const char* controller_ceiling = "1000";
  const char* planner_floor = "50000";
  const char* planner_ceiling = "100000";

  CLIENT_OS_MAX_PRIORITY = coordinator_os_priority - 1;
  CLIENT_OS_MIN_PRIORITY = coordinator_os_priority - 3;
  int client_os_priority_step = CLIENT_OS_MAX_PRIORITY - CLIENT_OS_MIN_PRIORITY;

  processor = boost::shared_ptr<processor_c>( new processor_c() );
  processor->name = "processor";

  dynamics = boost::shared_ptr<dynamics_c>( new dynamics_c( cpu_speed ) );
  dynamics->name = "dynamics";

  prey = boost::shared_ptr<timesink_c>( new timesink_c( scheduler_c::PROGRESS) );
  prey->name = "prey_timesink";
  prey->log = &log;
  //prey->priority = 0;

  prey_controller = boost::shared_ptr<osthread_c>( new osthread_c(&select, &read_notifications, &log) );
  prey_controller->name = "prey_controller";
  prey_controller->_max_os_priority = CLIENT_OS_MAX_PRIORITY;
  prey_controller->_min_os_priority = CLIENT_OS_MIN_PRIORITY;
  prey_controller->_os_priority_step = client_os_priority_step;
  prey_controller->_cpu_speed = cpu_speed;
  prey_controller->priority = 0;
///* 
  schedulererr = scheduler_c::create( prey_controller, 3, DEFAULT_CPU, "client-process", "prey-controller", "1", controller_floor, controller_ceiling );
//*/
  sprintf( spstr, "created prey-controller: pid[%d], _os_priority[%d]\n", prey_controller->pid, prey_controller->_os_priority );
  log_msg( spstr );

/*
  // Q: What is the priority of the predator controller vs. the planner?
  // I have heard two conflicting answers from Gabe.  Originally, he stated 
  // that the planner is higher than controller, but today he said opposite.
  // have to look into the blocking behavior and the measure of progress within
  // the predator timesink.
  // Q: How is this priority difference maintained in the two instances?
  // This priority is not system priority, but relative priority to one another
  // which is a critical distinction.  System priority gets adjusted, while
  // relative priority is static, but again this arbitrary distinction seems 
  // to conflict with what Gabe has said in the past as the only way the 
  // lower priority task gets exectuted is when the higher priority item gets
  // adjusted down.  This appears to imply that the block detection thread
  // runs at a priority level one below what is defined in this code and 
  // that the client min is one less than what is set above.
  pred = boost::shared_ptr<timesink_c>( new timesink_c(scheduler_c::PRIORITY) );
  pred->name = "pred_timesink";
  pred_controller = boost::shared_ptr<osthread_c>( new osthread_c(&select, &read_notifications, &log) );
  pred_controller->name = "pred_controller";
  pred_controller->_max_priority = CLIENT_MAX_PRIORITY - 1;
  pred_controller->_min_priority = CLIENT_MIN_PRIORITY - 1;
  pred_controller->_priority_step = client_priority_step;
  pred_controller->cpu_speed = cpu_speed;
  schedulererr = scheduler_c::create( pred_controller, 2, DEFAULT_CPU, "client-process", "pred-controller", "2", controller_floor, controller_ceiling );
  printf( "created pred-controller: pid[%d], priority[%d]\n", pred_controller->pid, pred_controller->priority );

  pred_planner = boost::shared_ptr<osthread_c>( new osthread_c(&select, &read_notifications, &log) );
  pred_planner->name = "pred_planner";
  pred_planner->_max_priority = CLIENT_MAX_PRIORITY;
  pred_planner->_min_priority = CLIENT_MIN_PRIORITY;
  pred_planner->_priority_step = client_priority_step;
  pred_planner->cpu_speed = cpu_speed;
  schedulererr = scheduler_c::create( pred_planner, 1, DEFAULT_CPU, "client-process", "pred-planner", "3", planner_floor, planner_ceiling );
  printf( "created pred-planner: pid[%d], priority[%d]\n", pred_planner->pid, pred_planner->priority );
*/
  processor->run_queue.push( dynamics );
  processor->run_queue.push( prey );
  //processor->run_queue.push( pred );

  prey->run_queue.push( prey_controller );

/*
  pred->run_queue.push( pred_planner );
  pred->run_queue.push( pred_controller );
*/

  // * initialize timer *
  if( timer.create( timer_sighandler, RTTIMER_SIGNAL ) != timer_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling timer.create(timer_sighandler,RTTIMER_SIGNAL)\nExiting\n" );
    log_msg( spstr );
    msgbuffer.close( );
    close_pipes( );
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

  sprintf( spstr, "timer events: actual[%d], caught[%u]\n", actual_timer_events, caught_timer_events );
  log_msg( spstr );

  printf( "shutting down\n" );
  log.flush();
  log.deallocate();

  // send notifications to clients
  kill( prey_controller->pid, SIGKILL );
  

  msgbuffer.close();
  close_pipes();
}

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] ) {
  // Due to realtime scheduling, et al, must have root access to run.
  if( geteuid() != 0 ) {
    sprintf( spstr, "This program requires root access.  Re-run with sudo.\nExiting\n" );
    printf( "%s", spstr );
    exit( 1 );
  }

  init( argc, argv );

  thread_p processor_thread = boost::dynamic_pointer_cast<thread_c>(processor);
  thread_p prey_thread = boost::dynamic_pointer_cast<thread_c>(prey);

  // last before main loop, arm the timer
  timer.arm( timer_c::PERIODIC, TIMER_PERIOD_NSECS );

  while( !quit ) {
    scheduler_c::step_system( processor_thread, processor->run_queue, processor->block_queue, &log );
    //select();
    //scheduler_c::step_system( prey_thread, prey->run_queue, prey->block_queue, &log );
  }

  shutdown();
  return 0;
}

