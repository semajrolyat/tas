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

//-----------------------------------------------------------------------------

// NEED TO SORT OUT HOW DYNAMICS CAN BE HANDLED THE SAME AS OTHER THREADS
//thread_c dynamics_thread;

//-----------------------------------------------------------------------------

thread_c wakeup_thread;

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

dynamics_plugin_c dynamics;

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
error_e init_pipes( void ) {
  int flags;
  int fd[2];

  // NOTE: a read channel needs to be blocking to force  block waiting for event
  // NOTE: a write channel should be non-blocking (won't ever fill up buffer anyway)

  // Open the prey timer to coordinator channel for timer events
  if( pipe( fd ) != 0 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling pipe(FD_PREY_TIMER_TO_COORDINATOR)\n" );
    error_log.write( errstr );
    return ERROR_FAILED;
  }
  flags = fcntl( fd[0], F_GETFL, 0 );
  fcntl( fd[0], F_SETFL, flags );
  flags = fcntl( fd[1], F_GETFL, 0 );
  fcntl( fd[1], F_SETFL, flags | O_NONBLOCK );

  if( dup2( fd[0], FD_TIMER_TO_COORDINATOR_READ_CHANNEL ) == -1 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling dup2(FD_TIMER_TO_COORDINATOR_READ_CHANNEL)\n" );
    error_log.write( errstr );
    close( fd[0] );
    close( fd[1] );
    return ERROR_FAILED;
  }
  if( dup2( fd[1], FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL ) == -1 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling dup2(FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL)\n" );
    error_log.write( errstr );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( fd[1] );
    return ERROR_FAILED;
  }

  // Open the wakeup to coordinator channel for wakeup (blocking) events
  if( pipe( fd ) != 0 ) {
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling pipe(FD_WAKEUP_TO_COORDINATOR)\n" );
    error_log.write( errstr );
    return ERROR_FAILED;
  }
  flags = fcntl( fd[0], F_GETFL, 0 );
  fcntl( fd[0], F_SETFL, flags );
  flags = fcntl( fd[1], F_GETFL, 0 );
  //fcntl( fd[1], F_SETFL, flags | O_NONBLOCK );
  fcntl( fd[1], F_SETFL, flags );

  if( dup2( fd[0], FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL ) == -1 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling dup2(FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL)\n" );
    error_log.write( errstr );
    close( fd[0] );
    close( fd[1] );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    return ERROR_FAILED;
  }

  if( dup2( fd[1], FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL ) == -1 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling dup2(FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL)\n" );
    error_log.write( errstr );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( fd[1] );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    return ERROR_FAILED;
  }

  // Open the coordinator to prey channel for notifications
  if( pipe( fd ) != 0 ) {
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling pipe(FD_COORDINATOR_TO_PREY)\n" );
    error_log.write( errstr );
    return ERROR_FAILED;
  }
  flags = fcntl( fd[0], F_GETFL, 0 );
  fcntl( fd[0], F_SETFL, flags );
  flags = fcntl( fd[1], F_GETFL, 0 );
  fcntl( fd[1], F_SETFL, flags | O_NONBLOCK );

  if( dup2( fd[0], FD_COORDINATOR_TO_PREY_READ_CHANNEL ) == -1 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling dup2(FD_COORDINATOR_TO_PREY_READ_CHANNEL)\n" );
    error_log.write( errstr );
    close( fd[0] );
    close( fd[1] );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    return ERROR_FAILED;
  }

  if( dup2( fd[1], FD_COORDINATOR_TO_PREY_WRITE_CHANNEL ) == -1 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling dup2(FD_COORDINATOR_TO_PREY_WRITE_CHANNEL)\n" );
    error_log.write( errstr );
    close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
    close( fd[1] );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    return ERROR_FAILED;
  }

  // Open the coordinator to predator channel for notifications
  if( pipe( fd ) != 0 ) {
    close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling pipe(FD_COORDINATOR_TO_PREDATOR)\n" );
    error_log.write( errstr );
    return ERROR_FAILED;
  }
  flags = fcntl( fd[0], F_GETFL, 0 );
  fcntl( fd[0], F_SETFL, flags );
  flags = fcntl( fd[1], F_GETFL, 0 );
  fcntl( fd[1], F_SETFL, flags | O_NONBLOCK );

  if( dup2( fd[0], FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL ) == -1 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling dup2(FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL)\n" );
    error_log.write( errstr );
    close( fd[0] );
    close( fd[1] );
    close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    return ERROR_FAILED;
  }

  if( dup2( fd[1], FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL ) == -1 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling dup2(FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL)\n" );
    error_log.write( errstr );
    close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL );
    close( fd[1] );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    return ERROR_FAILED;
  }

  // Open the coordinator to planner channel for notifications
  if( pipe( fd ) != 0 ) {
    close( FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling pipe(FD_COORDINATOR_TO_PREDATOR)\n" );
    error_log.write( errstr );
    return ERROR_FAILED;
  }
  flags = fcntl( fd[0], F_GETFL, 0 );
  fcntl( fd[0], F_SETFL, flags );
  flags = fcntl( fd[1], F_GETFL, 0 );
  fcntl( fd[1], F_SETFL, flags | O_NONBLOCK );

  if( dup2( fd[0], FD_COORDINATOR_TO_PLANNER_READ_CHANNEL ) == -1 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling dup2(FD_COORDINATOR_TO_PLANNER_READ_CHANNEL)\n" );
    error_log.write( errstr );
    close( fd[0] );
    close( fd[1] );
    close( FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    return ERROR_FAILED;
  }

  if( dup2( fd[1], FD_COORDINATOR_TO_PLANNER_WRITE_CHANNEL ) == -1 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling dup2(FD_COORDINATOR_TO_PLANNER_WRITE_CHANNEL)\n" );
    error_log.write( errstr );
    close( FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_READ_CHANNEL );
    close( fd[1] );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    return ERROR_FAILED;
  }

  // Open the prey to coordinator channel for notifications
  if( pipe( fd ) != 0 ) {
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_WRITE_CHANNEL );
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling pipe(FD_PREY_TO_COORDINATOR)\n" );
    error_log.write( errstr );
    return ERROR_FAILED;
  }
  flags = fcntl( fd[0], F_GETFL, 0 );
  fcntl( fd[0], F_SETFL, flags );
  flags = fcntl( fd[1], F_GETFL, 0 );
  fcntl( fd[1], F_SETFL, flags | O_NONBLOCK );

  if( dup2( fd[0], FD_PREY_TO_COORDINATOR_READ_CHANNEL ) == -1 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling dup2(FD_PREY_TO_COORDINATOR_READ_CHANNEL)\n" );
    error_log.write( errstr );
    close( fd[0] );
    close( fd[1] );
    close( FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_WRITE_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    return ERROR_FAILED;
  }

  if( dup2( fd[1], FD_PREY_TO_COORDINATOR_WRITE_CHANNEL ) == -1 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling dup2(FD_PREY_TO_COORDINATOR_WRITE_CHANNEL)\n" );
    error_log.write( errstr );
    close( FD_PREY_TO_COORDINATOR_READ_CHANNEL );
    close( fd[1] );
    close( FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_WRITE_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    return ERROR_FAILED;
  }

  // Open the predator to coordinator channel for notifications
  if( pipe( fd ) != 0 ) {
    close( FD_PREY_TO_COORDINATOR_READ_CHANNEL );
    close( FD_PREY_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_WRITE_CHANNEL );
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling pipe(FD_PREDATOR_TO_COORDINATOR)\n" );
    error_log.write( errstr );
    return ERROR_FAILED;
  }
  flags = fcntl( fd[0], F_GETFL, 0 );
  fcntl( fd[0], F_SETFL, flags );
  flags = fcntl( fd[1], F_GETFL, 0 );
  fcntl( fd[1], F_SETFL, flags | O_NONBLOCK );

  if( dup2( fd[0], FD_PREDATOR_TO_COORDINATOR_READ_CHANNEL ) == -1 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling dup2(FD_PREDATOR_TO_COORDINATOR_READ_CHANNEL)\n" );
    error_log.write( errstr );
    close( fd[0] );
    close( fd[1] );
    close( FD_PREY_TO_COORDINATOR_READ_CHANNEL );
    close( FD_PREY_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_WRITE_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    return ERROR_FAILED;
  }

  if( dup2( fd[1], FD_PREDATOR_TO_COORDINATOR_WRITE_CHANNEL ) == -1 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling dup2(FD_PREDATOR_TO_COORDINATOR_WRITE_CHANNEL)\n" );
    error_log.write( errstr );
    close( FD_PREDATOR_TO_COORDINATOR_READ_CHANNEL );
    close( fd[1] );
    close( FD_PREY_TO_COORDINATOR_READ_CHANNEL );
    close( FD_PREY_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_WRITE_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    return ERROR_FAILED;
  }

  // Open the planner to coordinator channel for notifications
  if( pipe( fd ) != 0 ) {
    close( FD_PREDATOR_TO_COORDINATOR_READ_CHANNEL );
    close( FD_PREDATOR_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_PREY_TO_COORDINATOR_READ_CHANNEL );
    close( FD_PREY_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_WRITE_CHANNEL );
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling pipe(FD_PLANNER_TO_COORDINATOR)\n" );
    error_log.write( errstr );
    return ERROR_FAILED;
  }
  flags = fcntl( fd[0], F_GETFL, 0 );
  fcntl( fd[0], F_SETFL, flags );
  flags = fcntl( fd[1], F_GETFL, 0 );
  fcntl( fd[1], F_SETFL, flags | O_NONBLOCK );

  if( dup2( fd[0], FD_PLANNER_TO_COORDINATOR_READ_CHANNEL ) == -1 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling dup2(FD_PLANNER_TO_COORDINATOR_READ_CHANNEL)\n" );
    error_log.write( errstr );
    close( fd[0] );
    close( fd[1] );
    close( FD_PREDATOR_TO_COORDINATOR_READ_CHANNEL );
    close( FD_PREDATOR_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_PREY_TO_COORDINATOR_READ_CHANNEL );
    close( FD_PREY_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_WRITE_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    return ERROR_FAILED;
  }

  if( dup2( fd[1], FD_PLANNER_TO_COORDINATOR_WRITE_CHANNEL ) == -1 ) {
    sprintf( errstr, "(coordinator.cpp) init_pipes() failed calling dup2(FD_PLANNER_TO_COORDINATOR_WRITE_CHANNEL)\n" );
    error_log.write( errstr );
    close( FD_PLANNER_TO_COORDINATOR_READ_CHANNEL );
    close( fd[1] );
    close( FD_PREDATOR_TO_COORDINATOR_READ_CHANNEL );
    close( FD_PREDATOR_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_PREY_TO_COORDINATOR_READ_CHANNEL );
    close( FD_PREY_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_READ_CHANNEL );
    close( FD_COORDINATOR_TO_PLANNER_WRITE_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
    close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
    close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
    return ERROR_FAILED;
  }
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Clean up the pipes to prevent leakage
void close_pipes( void ) {
  close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
  close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
  close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
  close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
  close( FD_COORDINATOR_TO_PREY_READ_CHANNEL );
  close( FD_COORDINATOR_TO_PREY_WRITE_CHANNEL );
  close( FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL );
  close( FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL );
  close( FD_COORDINATOR_TO_PLANNER_READ_CHANNEL );
  close( FD_COORDINATOR_TO_PLANNER_WRITE_CHANNEL );
  close( FD_PREY_TO_COORDINATOR_READ_CHANNEL );
  close( FD_PREY_TO_COORDINATOR_WRITE_CHANNEL );
  close( FD_PREDATOR_TO_COORDINATOR_READ_CHANNEL );
  close( FD_PREDATOR_TO_COORDINATOR_WRITE_CHANNEL );
  close( FD_PLANNER_TO_COORDINATOR_READ_CHANNEL );
  close( FD_PLANNER_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
// TIMER
//-----------------------------------------------------------------------------

/// The signal handler for the controller's timer.  Is called only when the controller's
/// budget is exhausted.  Puts a message out onto the comm channel the coordinator
/// is listening to for notifications about this event
void timer_sighandler( int signum, siginfo_t *si, void *data ) {

  timestamp_t ts;
  rdtscll( ts.cycle );

  if( write( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL, &ts, sizeof(timestamp_t) ) == -1) {
    std::string err_msg = "(coordinator.cpp) prey_timer_sighandler(...) failed making system call write(...)";
    error_log.write( error_string_bad_write( err_msg, errno ) );
    // TODO : determine if there is a need to recover
  }
}

//-----------------------------------------------------------------------------
// WAKEUP THREAD
//-----------------------------------------------------------------------------

/// the wakeup thread function itself.  Established at the priority level one
/// below the controller (therefore two below coordinator), this thread is unblocked when
/// the controller blocks and therefore can detect when the controller thread is not
/// consuming budget but is not directly suspended by the coordinator.

// TODO : should send a timestamp instead as the time between notification and message
// arrival may be significant
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
  if( scheduler.create( prey, "tas-ship-controller", 1, DEFAULT_PROCESSOR ) != SCHED_ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling scheduler.create(prey_thread,DEFAULT_PROCESSOR)\nExiting\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    amsgbuffer.close( );
    close_pipes( );
    error_log.close( );
    exit( 1 );
  }
  prey.PERIOD_NSEC = (unsigned long long)NSECS_PER_SEC / (unsigned long long)CONTROLLER_HZ;

  if( scheduler.create( pred, "tas-ship-controller", 1, DEFAULT_PROCESSOR ) != SCHED_ERROR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) init() failed calling scheduler.create(predator_thread,DEFAULT_PROCESSOR)\nExiting\n" );
    error_log.write( errstr );
    printf( "%s", errstr );
    amsgbuffer.close( );
    close_pipes( );
    error_log.close( );
    exit( 1 );
  }
  pred.PERIOD_NSEC = (unsigned long long)NSECS_PER_SEC / (unsigned long long)CONTROLLER_HZ;

  if( scheduler.create( planner, "tas-ship-planner", 1, DEFAULT_PROCESSOR ) != SCHED_ERROR_NONE ) {
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

  scheduler.shutdown( prey );
  scheduler.shutdown( pred );
  scheduler.shutdown( planner );
  scheduler.shutdown( dynamics );

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

error_e service_controller_actuator_msg( void ) {
  act_msg_c msg;
  char buf = 0;

  if( amsgbuffer.read( msg ) != BUFFER_ERR_NONE ) {
    sprintf( errstr, "(coordinator.cpp) main() failed calling sharedbuffer_c.read(msg)\n" );
    error_log.write( errstr );
    return ERROR_FAILED;
  }

  switch( msg.header.type ) {
  case MSG_COMMAND:
    dynamics_get_command( );
    break;
  case MSG_REQUEST:
    // Note: the child is blocking now on read, waiting for coordinator write
    // Handle requests in normal operation
    if( msg.header.requestor == REQUESTOR_PREY ) {
      if( prey.initialized ) {
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
      if( pred.initialized ) {
/*
        current_t = msg.state.time;
        dt = current_t - previous_t;
        previous_t = current_t;
*/
        //step_dynamics( dt ); // NO
      }
    } else if( msg.header.requestor == REQUESTOR_PLANNER ) {
      if( planner.initialized ) {
/*
        current_t = msg.state.time;
        dt = current_t - previous_t;
        previous_t = current_t;
*/
        //step_dynamics( dt ); // NO
      }
    }

    // Handle the initial request
    if( msg.header.requestor == REQUESTOR_PREY ) {
      if( !prey.initialized ) prey.initialized = true;
    } else if( msg.header.requestor == REQUESTOR_PREDATOR ) {
      if( !pred.initialized ) pred.initialized = true;
    } else if( msg.header.requestor == REQUESTOR_PLANNER ) {
      if( !planner.initialized ) planner.initialized = true;
    }

    dynamics_state_requested( );

    if( amsgbuffer.read( msg ) != BUFFER_ERR_NONE)  {
      sprintf( errstr, "(coordinator.cpp) main() failed calling actuator_msg_buffer_c.read(msg) while servicing ACTUATOR_MSG_REQUEST\n" );
      error_log.write( errstr );
      return ERROR_FAILED;
    }

    int write_channel;
    if( msg.header.requestor == REQUESTOR_PREY ) {
      write_channel = FD_COORDINATOR_TO_PREY_WRITE_CHANNEL;
    } else if( msg.header.requestor == REQUESTOR_PREDATOR ) {
      write_channel = FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL;
    } else if( msg.header.requestor == REQUESTOR_PLANNER ) {
      write_channel = FD_COORDINATOR_TO_PLANNER_WRITE_CHANNEL;
    }

    if( write( write_channel, &buf, 1 ) == -1 ) {
      std::string err_msg = "(coordinator.cpp) main() failed making system call write(...)";
      error_log.write( error_string_bad_write( err_msg, errno ) );
      // TODO : determine if there is a need to recover
      return ERROR_FAILED;
    }
    break;
  case MSG_NO_COMMAND:
  case MSG_REPLY:
  case MSG_UNDEFINED:
    break;
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

  controller_notification_c ctl_msg;
  fd_set fds_channels;
  int max_fd;

  init( argc, argv );

  max_fd = std::max( FD_TIMER_TO_COORDINATOR_READ_CHANNEL, FD_PREY_TO_COORDINATOR_READ_CHANNEL );
  max_fd = std::max( FD_TIMER_TO_COORDINATOR_READ_CHANNEL, FD_PREDATOR_TO_COORDINATOR_READ_CHANNEL );
  max_fd = std::max( FD_TIMER_TO_COORDINATOR_READ_CHANNEL, FD_PLANNER_TO_COORDINATOR_READ_CHANNEL );
  max_fd = std::max( max_fd, FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );

  // lock into memory to prevent pagefaults
  mlockall( MCL_CURRENT );

  printf( "starting main loop\n" );

  while( 1 ) {

    FD_ZERO( &fds_channels );
    FD_SET( FD_TIMER_TO_COORDINATOR_READ_CHANNEL, &fds_channels );
    FD_SET( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL, &fds_channels );
    FD_SET( FD_PREY_TO_COORDINATOR_READ_CHANNEL, &fds_channels );
    FD_SET( FD_PREDATOR_TO_COORDINATOR_READ_CHANNEL, &fds_channels );
    FD_SET( FD_PLANNER_TO_COORDINATOR_READ_CHANNEL, &fds_channels );

    if( select( max_fd + 1, &fds_channels, NULL, NULL, NULL ) ) {
      if( FD_ISSET( FD_TIMER_TO_COORDINATOR_READ_CHANNEL, &fds_channels ) ) {
        // timer event

        if( read( FD_TIMER_TO_COORDINATOR_READ_CHANNEL, &ts_timer_fired, sizeof(timestamp_t) ) == -1 ) {
          std::string err_msg = "(coordinator.cpp) main() failed making system call read(...) on FD_TIMER_TO_COORDINATOR_READ_CHANNEL";
          error_log.write( error_string_bad_read( err_msg, errno ) );
          // TODO : determine if there is a need to recover
        }

        //timestamp_timer_buffer.write( ts_timer_fired.cycle - ts_timer_armed.cycle );

        timestamp_t ts_dtimer;
        ts_dtimer.cycle = ts_timer_fired.cycle - ts_timer_armed.cycle;
        timestamp_timer_buffer.write( ts_dtimer );

        // Note : following is current exit strategy, e.g. run for a limited number of timer events
        if( ++timer_events == SIM_DURATION_CYCLES ) break;

        // controller needs to run!
        //scheduler.resume( controller_thread );
        // Before there was one controller.  Now need to schedule
  

      } else if( FD_ISSET( FD_PREY_TO_COORDINATOR_READ_CHANNEL, &fds_channels ) ) {
        // prey notification event

        if( read( FD_PREY_TO_COORDINATOR_READ_CHANNEL, &ctl_msg, sizeof( controller_notification_c ) ) == -1 ) {
          std::string err_msg = "(coordinator.cpp) main() failed making system call read(...) on FD_PREY_TO_COORDINATOR_READ_CHANNEL";
          error_log.write( error_string_bad_read( err_msg, errno ) );
          // TODO : determine if there is a need to recover
        }

        // check to see what kind of notification was sent.
        // if it's an actuator message then service it
        // else if it's a schedule request then schedule the controller timer
        if( ctl_msg.type == CONTROLLER_NOTIFICATION_ACTUATOR_EVENT ) {
          if( service_controller_actuator_msg( ) != ERROR_NONE) {
            sprintf( errstr, "(coordinator.cpp) main() failed calling service_controller_actuator_msg()\n" );
            error_log.write( errstr );
            break; //?
          }
        } else if( ctl_msg.type == CONTROLLER_NOTIFICATION_SNOOZE ) {
          // controller is preempted so it cannot run

          // set a timer, suspend the controller until the timer fires
          //controller_thread.suspend( );

          scheduler.suspend( prey );

          if( timer.arm( ctl_msg.ts, ts_timer_armed, prey.PERIOD_NSEC, cpu_speed_hz ) != timer_c::TIMER_ERROR_NONE ) {
            sprintf( errstr, "(coordinator.cpp) main() failed calling arm_timer()\n" );
            error_log.write( errstr );
            break; //?
          }
        }
      } else if( FD_ISSET( FD_PREDATOR_TO_COORDINATOR_READ_CHANNEL, &fds_channels ) ) {
        // predator notification event

        if( read( FD_PREDATOR_TO_COORDINATOR_READ_CHANNEL, &ctl_msg, sizeof( controller_notification_c ) ) == -1 ) {
          std::string err_msg = "(coordinator.cpp) main() failed making system call read(...) on FD_PREDATOR_TO_COORDINATOR_READ_CHANNEL";
          error_log.write( error_string_bad_read( err_msg, errno ) );
          // TODO : determine if there is a need to recover
        }

        // check to see what kind of notification was sent.
        // if it's an actuator message then service it
        // else if it's a schedule request then schedule the controller timer
        if( ctl_msg.type == CONTROLLER_NOTIFICATION_ACTUATOR_EVENT ) {
          if( service_controller_actuator_msg( ) != ERROR_NONE) {
            sprintf( errstr, "(coordinator.cpp) main() failed calling service_controller_actuator_msg()\n" );
            error_log.write( errstr );
            break; //?
          }
        } else if( ctl_msg.type == CONTROLLER_NOTIFICATION_SNOOZE ) {
          // controller is preempted so it cannot run

          // set a timer, suspend the controller until the timer fires
          //controller_thread.suspend( );

          scheduler.suspend( pred );

          if( timer.arm( ctl_msg.ts, ts_timer_armed, pred.PERIOD_NSEC, cpu_speed_hz ) != timer_c::TIMER_ERROR_NONE ) {
            sprintf( errstr, "(coordinator.cpp) main() failed calling arm_timer()\n" );
            error_log.write( errstr );
            break; //?
          }
        }
      } else if( FD_ISSET( FD_PLANNER_TO_COORDINATOR_READ_CHANNEL, &fds_channels ) ) {
        // planner notification event

        if( read( FD_PLANNER_TO_COORDINATOR_READ_CHANNEL, &ctl_msg, sizeof( controller_notification_c ) ) == -1 ) {
          std::string err_msg = "(coordinator.cpp) main() failed making system call read(...) on FD_PREDATOR_TO_COORDINATOR_READ_CHANNEL";
          error_log.write( error_string_bad_read( err_msg, errno ) );
          // TODO : determine if there is a need to recover
        }

        // check to see what kind of notification was sent.
        // if it's an actuator message then service it
        // else if it's a schedule request then schedule the controller timer
        if( ctl_msg.type == CONTROLLER_NOTIFICATION_ACTUATOR_EVENT ) {
          if( service_controller_actuator_msg( ) != ERROR_NONE) {
            sprintf( errstr, "(coordinator.cpp) main() failed calling service_controller_actuator_msg()\n" );
            error_log.write( errstr );
            break; //?
          }
        } else if( ctl_msg.type == CONTROLLER_NOTIFICATION_SNOOZE ) {
          // controller is preempted so it cannot run

          // set a timer, suspend the controller until the timer fires
          //controller_thread.suspend( );

          scheduler.suspend( planner );

          if( timer.arm( ctl_msg.ts, ts_timer_armed, planner.PERIOD_NSEC, cpu_speed_hz ) != timer_c::TIMER_ERROR_NONE ) {
            sprintf( errstr, "(coordinator.cpp) main() failed calling arm_timer()\n" );
            error_log.write( errstr );
            break; //?
          }
        }
      } else if( FD_ISSET( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL, &fds_channels ) ) {
        // wakeup event

        if( read( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL, &input_buffer, 1 ) == -1 ) {
          std::string err_msg = "(coordinator.cpp) main() failed making system call read(...) on FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL";
          error_log.write( error_string_bad_read( err_msg, errno ) );
          // TODO : determine if there is a need to recover
        }
      }
    }
  }
  munlockall();
  shutdown( );
}
