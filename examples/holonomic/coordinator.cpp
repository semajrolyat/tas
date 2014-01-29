/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

coordinator.cpp
-----------------------------------------------------------------------------*/

#include "tas.h"
#include "time.h"
#include "dynamics.h"
#include "cpu.h"
#include "schedule.h"
#include "log.h"
#include "experiment.h"

#include "thread.h"
#include "ipc.h"
#include "memory.h"

#include "controller.h"
#include "planner.h"
//-----------------------------------------------------------------------------

using boost::dynamic_pointer_cast;

//-----------------------------------------------------------------------------

timestamp_buffer_c timestamp_timer_buffer;

//-----------------------------------------------------------------------------

// Coordinator process information
static pid_t coordinator_pid;
static int coordinator_priority;

//-----------------------------------------------------------------------------

/*
timer_c controller_timer;
thread_c controller_thread;
bool controller_fully_initialized = false;
unsigned long long CONTROLLER_PERIOD_NSEC;
*/

timer_c timer;

/*
thread_c prey_thread;
bool prey_fully_initialized = false;
unsigned long long PREY_PERIOD_NSEC;

thread_c pred_thread;
bool pred_fully_initialized = false;
unsigned long long PRED_PERIOD_NSEC;

thread_c planner_thread;
bool planner_fully_initialized = false;
unsigned long long PLANNER_PERIOD_NSEC;
*/

controller_c prey;
controller_c pred;
planner_c planner;
dynamics_plugin_c dynamics;

thread_p prey_thread;
thread_p pred_thread;
thread_p planner_thread;
thread_p dynamics_thread;

bool system_initialized = false;

pipe_c pipe_timer;
pipe_c pipe_wakeup;
//-----------------------------------------------------------------------------

// NEED TO SORT OUT HOW DYNAMICS CAN BE HANDLED THE SAME AS OTHER THREADS
//thread_c dynamics_thread;

//-----------------------------------------------------------------------------

thread_p wakeup_thread;

//-----------------------------------------------------------------------------
// SCHEDULER
//-----------------------------------------------------------------------------

scheduler_c scheduler;

//-----------------------------------------------------------------------------
// CPU
//-----------------------------------------------------------------------------

#define DEFAULT_PROCESSOR    0

//-----------------------------------------------------------------------------

unsigned long long cpu_speed_hz;

//-----------------------------------------------------------------------------
// LOGGING
//-----------------------------------------------------------------------------

log_c variance_log;
log_c error_log;
char errstr[ 256 ];

//-----------------------------------------------------------------------------

error_e open_error_log( void ) {
  error_log = log_c( "error.log" );
  //error_log = log_c( LOG_CHANNEL_STDERR );

  if( error_log.open( ) != LOG_ERROR_NONE ) {
    printf( "(coordinator.cpp) ERROR: open_error_log() failed calling log_c.open() on file error.log\n" );
    return ERROR_FAILED;
  }

  // dup the error log fd into a known channel so shared resources
  // know exactly what fd the log file is using
  if( dup2( error_log.get_fd(), FD_ERROR_LOG ) == -1 ) {
    printf( "(coordinator.cpp) ERROR: open_error_log() failed calling dup2(error_log.get_fd(), FD_ERROR_LOG)\n" );
    return ERROR_FAILED;
  }

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------

error_e open_variance_log( void ) {
  variance_log = log_c( "variance.dat" );
  //variance_log = log_c( LOG_CHANNEL_STDOUT );

  if( variance_log.open( ) != LOG_ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) open_variance_log() failed calling log_c.open() on file variance.dat\n" );
    error_log.write( errstr );
    return ERROR_FAILED;
  }
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
// SHARED RESOURCE MANAGEMENT
//-----------------------------------------------------------------------------

// The shared message buffer
sharedbuffer_c amsgbuffer;

//-----------------------------------------------------------------------------
// PLUGIN MANAGEMENT
//-----------------------------------------------------------------------------

// list of plugins.
// Note: In actuality only support a single plugin at this time.
//std::list< dynamics_plugin_c > plugins;


//-----------------------------------------------------------------------------

/**
  invokes the intialization function (init_f) of all registered dynamics plugins by
  forwarding command line parameters to the function
*/
/*
void coordinator_init_dynamics( int argc, char** argv ) {
    for( std::list< DynamicsPlugin >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
        (*it->init)( argc, argv );
    }
}
*/
//-----------------------------------------------------------------------------

/**
  invokes the step function (step_f) of all the registered dynamics plugins
*/
void step_dynamics( double dt ) {
/*
  for( std::list< dynamics_plugin_c >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
    (*it->step)( dt );
  }
*/
  dynamics.step( dt );
}

//-----------------------------------------------------------------------------
void shutdown_dynamics( void ) {
/*
  for( std::list< dynamics_plugin_c >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
    (*it->shutdown)( );
  }
*/
  dynamics.shutdown();
}

//-----------------------------------------------------------------------------

/**
  a means to publish the state from dynamics is necessary (at least in this architecture)
  because there is a need to seed data to the controller (prime the controller)
  so everything can start.  Once everything is started though, publish state is
  really wrapped into the run_dynamics mechanism
*/
void dynamics_state_requested( void ) {
/*
  for( std::list< dynamics_plugin_c >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
    (*it->write_state)( );
  }
*/
  dynamics.write_state();
}

//-----------------------------------------------------------------------------

void dynamics_get_command( void ) {
/*
  for( std::list< dynamics_plugin_c >::iterator it = plugins.begin(); it != plugins.end(); it++ ) {
    (*it->read_command)( );
  }
*/
  dynamics.read_command();
}

//-----------------------------------------------------------------------------
// TODO : Improve error handling
error_e init_dynamics( const int& argc, char* argv[] ) {
/*
  dynamics_plugin_c plugin;
  int result = plugin.read( DYNAMICS_PLUGIN );	// TODO: Sanity/Safety checking
  plugin.init( argc, argv );

  // if both functions were located then push the pair into the list of plugins
  if( !result ) plugins.push_back( plugin );
*/

  // Note: !Refactor to check the error condition on both read and init!

  int result = dynamics.read( DYNAMICS_PLUGIN );  // TODO: Sanity/Safety checking
  dynamics.init( argc, argv );

  // if both functions were located then push the pair into the list of plugins
  //if( !result ) plugins.push_back( plugin );
  
  return ERROR_NONE;
}


//-----------------------------------------------------------------------------
// INTERPROCESS COMMUNICATION (IPC)
//-----------------------------------------------------------------------------

/// Initializes all interprocess communication channels.  This must be done
/// before any controller processes are forked or threads are spawned so that they
/// will inherit the file descriptors from the coordinator.
// NOTE: a read channel needs to be blocking to force  block waiting for event
// NOTE: a write channel should be non-blocking (won't ever fill up buffer anyway).  But wakeup is set as an exception for now.
error_e init_pipes( void ) {

  pipe_c::pipe_error_e r;

  // open timer channels
  pipe_timer = pipe_c( FD_TIMER_TO_COORDINATOR_READ_CHANNEL, FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
  r = pipe_timer.open();
  if( r != pipe_c::PIPE_ERROR_NONE ) {
    pipe_c::log_error( error_log, r, "coordinator.cpp", "init_pipes", "pipe_timer.open()" );
    return ERROR_FAILED;
  }

  // open wakeup channels
  pipe_wakeup = pipe_c( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL, FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL, true );
  r = pipe_wakeup.open();
  if( r != pipe_c::PIPE_ERROR_NONE ) {
    pipe_c::log_error( error_log, r, "coordinator.cpp", "init_pipes", "pipe_wakeup.open()" );
    pipe_timer.close();
    return ERROR_FAILED;
  }

  // open prey comm to channels
  prey.pipe_to = pipe_c( FD_COORDINATOR_TO_PREY_READ_CHANNEL, FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
  r = prey.pipe_to.open();
  if( r != pipe_c::PIPE_ERROR_NONE ) {
    pipe_c::log_error( error_log, r, "coordinator.cpp", "init_pipes", "prey.pipe_to.open()" );
    pipe_timer.close();
    pipe_wakeup.close();
    return ERROR_FAILED;
  }

  // open prey comm out channels
  prey.pipe_from = pipe_c( FD_PREY_TO_COORDINATOR_READ_CHANNEL, FD_PREY_TO_COORDINATOR_WRITE_CHANNEL );
  r = prey.pipe_from.open();
  if( r != pipe_c::PIPE_ERROR_NONE ) {
    pipe_c::log_error( error_log, r, "coordinator.cpp", "init_pipes", "prey.pipe_from.open()" );
    pipe_timer.close();
    pipe_wakeup.close();
    prey.pipe_to.close();
    return ERROR_FAILED;
  }

  // open pred comm in channels
  pred.pipe_to = pipe_c( FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL, FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL );
  r = pred.pipe_to.open();
  if( r != pipe_c::PIPE_ERROR_NONE ) {
    pipe_c::log_error( error_log, r, "coordinator.cpp", "init_pipes", "pred.pipe_to.open()" );
    pipe_timer.close();
    pipe_wakeup.close();
    prey.pipe_to.close();
    prey.pipe_from.close();
    return ERROR_FAILED;
  }

  // open pred comm out channels
  pred.pipe_from = pipe_c( FD_PREDATOR_TO_COORDINATOR_READ_CHANNEL, FD_PREDATOR_TO_COORDINATOR_WRITE_CHANNEL );
  r = pred.pipe_from.open();
  if( r != pipe_c::PIPE_ERROR_NONE ) {
    pipe_c::log_error( error_log, r, "coordinator.cpp", "init_pipes", "pred.pipe_from.open()" );
    pipe_timer.close();
    pipe_wakeup.close();
    prey.pipe_to.close();
    prey.pipe_from.close();
    pred.pipe_to.close();
    return ERROR_FAILED;
  }

  // open planner comm to channels
  planner.pipe_to = pipe_c( FD_COORDINATOR_TO_PLANNER_READ_CHANNEL, FD_COORDINATOR_TO_PLANNER_WRITE_CHANNEL );
  r = planner.pipe_to.open();
  if( r != pipe_c::PIPE_ERROR_NONE ) {
    pipe_c::log_error( error_log, r, "coordinator.cpp", "init_pipes", "planner.pipe_to.open()" );
    pipe_timer.close();
    pipe_wakeup.close();
    prey.pipe_to.close();
    prey.pipe_from.close();
    pred.pipe_to.close();
    pred.pipe_from.close();
    return ERROR_FAILED;
  }

  // open planner comm out channels
  planner.pipe_from = pipe_c( FD_PLANNER_TO_COORDINATOR_READ_CHANNEL, FD_PLANNER_TO_COORDINATOR_WRITE_CHANNEL );
  r = planner.pipe_from.open();
  if( r != pipe_c::PIPE_ERROR_NONE ) {
    pipe_c::log_error( error_log, r, "coordinator.cpp", "init_pipes", "planner,pipe_from.open()" );
    pipe_timer.close();
    pipe_wakeup.close();
    prey.pipe_to.close();
    prey.pipe_from.close();
    pred.pipe_to.close();
    pred.pipe_from.close();
    planner.pipe_to.close();
    return ERROR_FAILED;
  }

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Clean up the pipes to prevent leakage
void close_pipes( void ) {
  pipe_timer.close();
  pipe_wakeup.close();
  prey.pipe_to.close();
  prey.pipe_from.close();
  pred.pipe_to.close();
  pred.pipe_from.close();
  planner.pipe_to.close();
  planner.pipe_from.close();
}

//-----------------------------------------------------------------------------
// TIMER
//-----------------------------------------------------------------------------

/// The signal handler for the timer.  Is called only when the user process'
/// budget is exhausted.  Puts a timestamp message out onto the comm channel 
/// that wakes the coordinator.
void timer_sighandler( int signum, siginfo_t *si, void *data ) {
 
  notification_c notification;
  notification.type = NOTIFICATION_TIMER;
  notification.ts = generate_timestamp();

  pipe_c::pipe_error_e r = pipe_timer.write( notification );
  if( r != pipe_c::PIPE_ERROR_NONE ) {
  std::string err_msg = "(coordinator.cpp) timer_sighandler(...) failed making system call write(...)";
    error_log.write( error_string_bad_write( err_msg, errno ) );
    // TODO : determine if there is a need to recover
  }
}

//-----------------------------------------------------------------------------
// WAKEUP THREAD
//-----------------------------------------------------------------------------

/// the wakeup thread function itself.  Established at the priority level one
/// below the controller (therefore two below coordinator), this thread is unblocked when
/// the controller blocks and therefore can detect when the controller thread is/// not consuming budget but is not directly suspended by the coordinator.

void* wakeup( void* ) {
  char buf;
  timespec ts_sleep, ts_rem;
  ts_sleep.tv_sec = 0;
  ts_sleep.tv_nsec = 100;

  notification_c notification;
  notification.type = NOTIFICATION_WAKEUP;

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

    notification.ts = generate_timestamp();

    pipe_c::pipe_error_e r = pipe_wakeup.write( notification );
    if( r != pipe_c::PIPE_ERROR_NONE ) {
      std::string err_msg = "(coordinator.cpp) wakeup() failed making system call write(...)";
      error_log.write( error_string_bad_write( err_msg, errno ) );
      // TODO : determine if there is a need to recover
    }

*/
    clock_nanosleep( CLOCK_MONOTONIC, 0, &ts_sleep, &ts_rem );
  }
  return NULL;
}

//-----------------------------------------------------------------------------

void init( int argc, char* argv[] ) {

  if( open_error_log( ) != ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling open_error_log().\nExiting\n" );
    printf( "%s", errstr );
    exit( 1 );
  }

  coordinator_pid = getpid( );

  //++++++++++++++++++++++++++++++++++++++++++++++++

  if( set_cpu( coordinator_pid, DEFAULT_PROCESSOR ) != ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling set_cpu(coordinator_pid,DEFAULT_PROCESSOR).\nExiting\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    error_log.close( );
    exit( 1 );
  }

  if( set_realtime_schedule_max( coordinator_pid, coordinator_priority ) != ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling set_realtime_schedule_max(coordinator_pid,coordinator_priority).\nExiting\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    error_log.close( );
    exit( 1 );
  }

  printf( "coordinator process priority: %d\n", coordinator_priority );

  //++++++++++++++++++++++++++++++++++++++++++++++++
  // Query the underlying system for information about the environment

  // Determine if the OS supports high resolution timers
  struct timespec res;
  clock_getres( CLOCK_MONOTONIC, &res );
  double clock_res = timespec_to_real( res );
  printf( "Clock resolution (secs): %10.9f\n", clock_res );

  if( res.tv_sec != 0 && res.tv_nsec != 1 ) {
    sprintf( errstr, "(coordinator.cpp) init() failed.  The host operating system does not support high resolution timers.  Consult your system documentation on enabling this feature.\nExiting\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    error_log.close( );
    exit( 1 );
  }

  // Get the cpu speed
  if( get_cpu_frequency( cpu_speed_hz, DEFAULT_PROCESSOR ) != CPUINFO_ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling read_cpuinfo()\nExiting\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    error_log.close( );
    exit( 1 );
  }

  //++++++++++++++++++++++++++++++++++++++++++++++++
  // Initialize any shared components before initializing anything
  // else that might use them

  // Initialize IPC channels
  if( init_pipes( ) !=  ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling init_pipes()\nExiting\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    error_log.close( );
    exit( 1 );
  }
  // Note : From here on out, if error, close pipes

  // Create and initialize the actuator message buffer
  amsgbuffer = sharedbuffer_c( "/amsgbuffer", true );
  if( amsgbuffer.open( ) != BUFFER_ERR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling actuator_msg_buffer_c.open(...,true)\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    close_pipes( );
    error_log.close( );
    exit( 1 );
  }
  // Note : From here on out, if error, close the buffer

  //++++++++++++++++++++++++++++++++++++++++++++++++
  // initialize dynamics

  if( init_dynamics( argc, argv ) != ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling init_dynamics(argc,argv)\nExiting\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    amsgbuffer.close( );
    close_pipes( );
    error_log.close( );
    exit( 1 );
  }

  //++++++++++++++++++++++++++++++++++++++++++++++++
  // initialize planners and controllers

  // Note: pass serialization args here
  prey_thread = thread_p( new controller_c() );
  if( scheduler.create( prey_thread, "tas-ship-controller", 1, DEFAULT_PROCESSOR ) != SCHED_ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling scheduler.create(prey_thread,DEFAULT_PROCESSOR)\nExiting\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    amsgbuffer.close( );
    close_pipes( );
    error_log.close( );
    exit( 1 );
  }
  prey.PERIOD_NSEC = (unsigned long long)NSECS_PER_SEC / (unsigned long long)CONTROLLER_HZ;

  // Note: pass serialization args here
  pred_thread = thread_p( new controller_c() );
  if( scheduler.create( pred_thread, "tas-ship-controller", 1, DEFAULT_PROCESSOR ) != SCHED_ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling scheduler.create(predator_thread,DEFAULT_PROCESSOR)\nExiting\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    amsgbuffer.close( );
    close_pipes( );
    error_log.close( );
    exit( 1 );
  }
  pred.PERIOD_NSEC = (unsigned long long)NSECS_PER_SEC / (unsigned long long)CONTROLLER_HZ;

  // Note: pass serialization args here
  planner_thread = thread_p( new planner_c() );
  if( scheduler.create( planner_thread, "tas-ship-planner", 1, DEFAULT_PROCESSOR ) != SCHED_ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling scheduler.create(planner_thread,DEFAULT_PROCESSOR)\nExiting\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    amsgbuffer.close( );
    close_pipes( );
    error_log.close( );
    exit( 1 );
  }
  planner.PERIOD_NSEC = (unsigned long long)NSECS_PER_SEC / (unsigned long long)CONTROLLER_HZ;

  //++++++++++++++++++++++++++++++++++++++++++++++++
  // intialize all other monitor facilities

  printf( "(coordinator) setting initial timer\n" );

  if( timer.create( timer_sighandler, RTTIMER_SIGNAL ) != timer_c::TIMER_ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling prey_timer.create(timer_sighandler,RTTIMER_SIGNAL)\nExiting\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    amsgbuffer.close( );
    close_pipes( );
    error_log.close( );
    exit( 1 );
  }

  // initialize the wakeup thread
  wakeup_thread = thread_p( new thread_c() );
  if( scheduler.create( wakeup_thread, coordinator_priority - 2, wakeup ) != SCHED_ERROR_NONE ) {
    amsgbuffer.close( );
    close_pipes( );
    error_log.close( );
    printf( "(coordinator.cpp) init() failed calling wakeup_thread.create(...,wakeup)\nExiting\n" );
    exit( 1 );
  }

  //++++++++++++++++++++++++++++++++++++++++++++++++
  // open any other logs

/*
  // open the variance log
  if( open_variance_log( ) != ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling open_variance_log()\nExiting\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    amsgbuffer.close( );
    close_pipes( );
    error_log.close( );
    exit( 1 );
  }
*/

  timestamp_timer_buffer = timestamp_buffer_c( cpu_speed_hz, "timestamps_timer.dat" );
  if( timestamp_timer_buffer.open( ) != ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling timestamp_buffer_c.open()\nExiting\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    amsgbuffer.close( );
    close_pipes( );
    error_log.close( );
    exit( 1 );
  }

  //********** SOME TESTING *****************
/*
  // Validated : thread_c.set_priority can successfully adjust priority for a PROCESS
  if( controller_thread.set_priority( coordinator_priority - 2 ) == ERROR_FAILED ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling controller_thread.set_priority(...)\nExiting\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    amsgbuffer.close( );
    close_pipes( );
    error_log.close( );
    exit( 1 );
  }

  // Validated : thread_c.set_priority can successfully adjust priority for a THREAD
  if( wakeup_thread.set_priority( coordinator_priority - 3 ) == ERROR_FAILED ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling wakeup_thread.set_priority(...)\nExiting\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    amsgbuffer.close( );
    close_pipes( );
    error_log.close( );
    exit( 1 );
  }
*/
}

//-----------------------------------------------------------------------------

void shutdown( void ) {
  printf( "shutting down\n" );

  //variance_log.close( );

  //pthread_join( wakeup_thread, NULL );
  //pthread_kill( wakeup_thread, SIGKILL );

  scheduler.shutdown( prey_thread );
  scheduler.shutdown( pred_thread );
  scheduler.shutdown( planner_thread );
  scheduler.shutdown( dynamics_thread );

  sleep( 1 );  // block for a second to allow controller to receive signal

  //shutdown_dynamics( );

  amsgbuffer.close( );
  close_pipes( );

  timestamp_timer_buffer.close();

  // Note: can get a double free exception if the log is closed in this process
  // before it is closed in the controller

  error_log.close( );

  //exit( 0 );
}

//-----------------------------------------------------------------------------

double previous_t = 0.0;
double current_t = 0.0;
double dt;

//-----------------------------------------------------------------------------

error_e service_process_notification( void ) {
  act_msg_c msg;
  notification_c note;

  if( amsgbuffer.read( msg ) != BUFFER_ERR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) main() failed calling sharedbuffer_c.read(msg)\n" );
    error_log.write( errstr );
    return ERROR_FAILED;
  }

  if( msg.header.type == MSG_COMMAND ) {
    dynamics_get_command( );

    // mark as initialized once initial command is delivered
    if( msg.header.requestor == REQUESTOR_PREY ) {
      prey_thread->initialized = true;
    } else if( msg.header.requestor == REQUESTOR_PREDATOR ) {
      pred_thread->initialized = true;
    } else if( msg.header.requestor == REQUESTOR_PLANNER ) {
      planner_thread->initialized = true;
    }
  } else if( msg.header.type == MSG_REQUEST ) {
    // Note: the child is blocking now on read, waiting for coordinator write
    // Handle requests in normal operation
    if( msg.header.requestor == REQUESTOR_PREY ) {
      if( prey_thread->initialized ) {
	// NOTE: The commented code in these branches needs attention
/*
        current_t = msg.state.time;
        dt = current_t - previous_t;
        previous_t = current_t;
*/
        //step_dynamics( dt ); // NO

        // Refactor of dynamics.  Use something like the following instead of above.  Really should be dispatch by scheduler
        // dynamics.step_size = dt;
        // dynamics.execute()
      }
    } else if( msg.header.requestor == REQUESTOR_PREDATOR ) {
      if( pred_thread->initialized ) {
/*
        current_t = msg.state.time;
        dt = current_t - previous_t;
        previous_t = current_t;
*/
        //step_dynamics( dt ); // NO
      }
    } else if( msg.header.requestor == REQUESTOR_PLANNER ) {
      if( planner_thread->initialized ) {
/*
        current_t = msg.state.time;
        dt = current_t - previous_t;
        previous_t = current_t;
*/
        //step_dynamics( dt ); // NO
      }
    }

    dynamics_state_requested( );

    if( amsgbuffer.read( msg ) != BUFFER_ERR_NONE)  {
      sprintf( errstr, "(coordinator.cpp) main() failed calling actuator_msg_buffer_c.read(msg) while servicing ACTUATOR_MSG_REQUEST\n" );
      error_log.write( errstr );
      return ERROR_FAILED;
    }

    pipe_c* pipe;
    if( msg.header.requestor == REQUESTOR_PREY ) 
      pipe = &prey.pipe_to;
    else if( msg.header.requestor == REQUESTOR_PREDATOR )
      pipe = &pred.pipe_to;
    else if( msg.header.requestor == REQUESTOR_PLANNER )
      pipe = &planner.pipe_to;

    note.type = NOTIFICATION_RESPONSE;
    note.ts = generate_timestamp();

    pipe_c::pipe_error_e r = pipe->write( note );
    if( r != pipe_c::PIPE_ERROR_NONE ) {
      std::string err_msg = "(coordinator.cpp) main() failed making system call write(...)";
      error_log.write( error_string_bad_write( err_msg, errno ) );
      return ERROR_FAILED;
    }
  } else if( msg.header.type == MSG_NO_COMMAND ) {
  } else if( msg.header.type == MSG_REPLY ) {
  } else if( msg.header.type == MSG_UNDEFINED ) {
  }
  return ERROR_NONE;
}


//-----------------------------------------------------------------------------
// ENTRY POINT
//-----------------------------------------------------------------------------

int main( int argc, char* argv[] ) {

  // Due to realtime scheduling, et al, this program must have root access to run.
  if( geteuid() != 0 ) {
    sprintf( errstr, "This program requires root access.  Re-run with sudo.\nExiting\n" );
    printf( "%s", errstr );
    exit( 1 );
  }

  char input_buffer;
  unsigned int timer_events = 0;
  timestamp_t ts_timer_armed, ts_timer_fired;

  fd_set fds_channels;
  int max_fd;

  notification_c notification;
  pipe_c::pipe_error_e pipe_err;

  init( argc, argv );

  scheduler.schedule( dynamics_thread );

  max_fd=std::max( pipe_timer.fd_read_channel, prey.pipe_from.fd_read_channel );
  max_fd=std::max( max_fd, pred.pipe_from.fd_read_channel );
  max_fd=std::max( max_fd, planner.pipe_from.fd_read_channel );
  max_fd=std::max( max_fd, pipe_wakeup.fd_read_channel );

  // lock into memory to prevent pagefaults
  mlockall( MCL_CURRENT );

  printf( "starting main loop\n" );

  while( 1 ) {

    FD_ZERO( &fds_channels );
    FD_SET( pipe_timer.fd_read_channel, &fds_channels );
    FD_SET( pipe_wakeup.fd_read_channel, &fds_channels );
    FD_SET( prey.pipe_from.fd_read_channel, &fds_channels );
    FD_SET( pred.pipe_from.fd_read_channel, &fds_channels );
    FD_SET( planner.pipe_from.fd_read_channel, &fds_channels );

    if( select( max_fd + 1, &fds_channels, NULL, NULL, NULL ) ) {
      if( FD_ISSET( pipe_timer.fd_read_channel, &fds_channels ) ) {
        // timer event
   
        pipe_err = pipe_timer.read( notification );
        if( pipe_err != pipe_c::PIPE_ERROR_NONE ) {
          std::string err_msg = "(coordinator.cpp) main() failed making system call read(...) on FD_TIMER_TO_COORDINATOR_READ_CHANNEL";
          error_log.write( error_string_bad_read( err_msg, errno ) );
          // TODO : determine if there is a need to recover
        }

        // if not all threads are initialized, ignore this notification
        if( !system_initialized ) continue;
        // else if all threads are initialized, proceed

        // if the scheduler has no active thread (first time here)
        if( scheduler.current_thread ) {
          // dispatch next thread

          // dequeue next thread as active thread
          scheduler.current_thread = scheduler.next();
          // zero out the number of quantums for the newly active thread
          scheduler.quantum = 0;
          // execute the current thread
          scheduler.current_thread->execute();
        } else {
          // else the scheduler has an active thread(only after first time here)

          // increment the number of quantums the active thread has run for
          scheduler.quantum++;

          // if the active thread's budget iis exhausted
          if( scheduler.quantum == scheduler.current_thread->budget ) {
            // if not dynamics (as it is a special case)
            if( scheduler.current_thread.get() != dynamics_thread.get() ) {
              // explicitly block the active thread
              scheduler.suspend( scheduler.current_thread );
            }
            // reschedule thread
            scheduler.schedule( scheduler.current_thread );
          }

          // dispatch next thread

          // dequeue next thread as active thread
          scheduler.current_thread = scheduler.next();
          // zero out the number of quantums for the newly active thread
          scheduler.quantum = 0;
          // execute the current thread
          scheduler.current_thread->execute();
          // *dynamics calls step(dt), other thread's call resume()
        }

      } else if( FD_ISSET( prey.pipe_from.fd_read_channel, &fds_channels ) ) {
        // prey notification event

        pipe_err = pipe_timer.read( notification );
        if( pipe_err != pipe_c::PIPE_ERROR_NONE ) {
          std::string err_msg = "(coordinator.cpp) main() failed making system call read(...) on FD_PREY_TO_COORDINATOR_READ_CHANNEL";
          error_log.write( error_string_bad_read( err_msg, errno ) );
          // TODO : determine if there is a need to recover
        }

        // check to see what kind of notification was sent.
        if( notification.type == NOTIFICATION_UNDEFINED ) {
          // unknown issue
        } else if( notification.type == NOTIFICATION_REQUEST || 
                   notification.type == NOTIFICATION_RESPONSE ) {
          // requested state or issued a command
          if( service_process_notification( ) != ERROR_NONE) {
            sprintf( errstr, "(coordinator.cpp) main() failed calling service_controller_actuator_msg()\n" );
            error_log.write( errstr );
            break; //?
          }
        } else if( notification.type == NOTIFICATION_IDLE ) {

          // if the active thread is not the prey thread, ignore
          if( scheduler.current_thread.get() != prey_thread.get() ) continue;

          // otherwise suspend so it cannot runaway on a coordindator block
          scheduler.suspend( scheduler.current_thread );
          // reschedule active thread
          scheduler.schedule( scheduler.current_thread );

          // dispatch next thread

          // dequeue next thread as active thread
          scheduler.current_thread = scheduler.next();
          // zero out the number of quantums for the newly active thread
          scheduler.quantum = 0;
          // execute the current thread.  can only be user process.  dynamics
          // can never get here
          scheduler.current_thread->execute();
        }

        // if all components are initialized mark system as initialized
        if( prey_thread->initialized && pred_thread->initialized && 
            planner_thread->initialized && dynamics_thread->initialized )
          system_initialized = true;

      // TODO : Add in the predator and planner once prey is worked out
      } else if( FD_ISSET( pipe_wakeup.fd_read_channel, &fds_channels ) ) {
        // wakeup event

        pipe_err = pipe_wakeup.read( notification );
        if( pipe_err != pipe_c::PIPE_ERROR_NONE ) {
          std::string err_msg = "(coordinator.cpp) main() failed making system call read(...) on FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL";
          error_log.write( error_string_bad_read( err_msg, errno ) );
          // TODO : determine if there is a need to recover
        }

        // Dont really have to check type of notification here because the
        // only notifications that come on this pipe are wakeups.
      }
    }
  }
  munlockall();
  shutdown( );
}
