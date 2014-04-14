#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <atomic>

#include <unistd.h>
#include <signal.h>

#include "../time.h"
#include "../os.h"
#include "../notification.h"

#include "../channels.h"

#include "../osthread.h"

#include <errno.h>

//-----------------------------------------------------------------------------
// Local helpers
static osthread_p     mythread;
//std::atomic<int>      quit;
static bool           quit;

//-----------------------------------------------------------------------------
// Notification Overhead
static int            write_fd;
static int            read_fd;

notification_t        command_note;
notification_t        state_note;
notification_t        block_note;
notification_t        server_note;

//-----------------------------------------------------------------------------
// Work simulation
volatile unsigned int counter;
static unsigned       rand_delta;
static unsigned       rand_seed;
static unsigned       min_cycles;
static unsigned       max_cycles;

//-----------------------------------------------------------------------------
// read( blocks process ) the notification sent back from the coordinator
void read( notification_t& note ) {
  if( __read( read_fd, &note, sizeof(notification_t) ) == -1 ) {
    // ERROR
    //return false;
  }
}

//-----------------------------------------------------------------------------
void write( notification_t& note ) {

  // update the timestamp on the note
  note.ts = generate_timestamp();

  if( __write( write_fd, &note, sizeof(notification_t) ) == -1 ) {
/*
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
    err = "(" + mythread.name + ") failed making system call write(...)" + eno;
    printf( "ERROR: %s\n", err.c_str() );
*/
  }
}

//-----------------------------------------------------------------------------
void term_sighandler( int signum ) {
  quit = true;
  //quit.store( 1, std::memory_order_seq_cst );
}

//-----------------------------------------------------------------------------
void compute_command( void ) {
  counter = 0;
  unsigned cycles_to_run = (rand() % rand_delta) + min_cycles;

  for( unsigned i = 0; i < cycles_to_run; i++ ) {
    counter++;
  }

}

//-----------------------------------------------------------------------------
// Request/Reply
void request_state( void ) {

  // write the request information into the shared memory buffer

  // send the notification to the coordinator
  write( state_note );

  // read( block ) the notification sent back from the coordinator
  read( server_note );

  // read data from the shared memory buffer

  
  //return true;
}

//-----------------------------------------------------------------------------
// Pub/Sub
bool publish_command( void ) {

  // put data in the shared memory buffer

  // send the notification to the coordinator
  write( command_note );
  
  return true;
}

//-----------------------------------------------------------------------------
void publish_block( void ) {

  // send the notification to the coordinator
  write( block_note );

}

//-----------------------------------------------------------------------------
int init( int argc, char* argv[] ) {
  mythread = osthread_p( new osthread_c( argv[0] ) );

  quit = false;
  //quit.store( 0, std::memory_order_seq_cst );
  mythread->pid = getpid();

  // * install SIGTERM signal handler *
  struct sigaction action;
  memset( &action, 0, sizeof(struct sigaction) );
  action.sa_handler = term_sighandler;
  sigaction( SIGTERM, &action, NULL );

  if( argc < 4 ) {
    printf( "ERROR: client process[%s] was not given enough arguments\nExiting client process.\n", mythread->name );
    exit( 1 );
  }
  rand_seed = (unsigned) atoi( argv[1] );
  min_cycles = (unsigned) atoi( argv[2] );
  max_cycles = (unsigned) atoi( argv[3] );

  //printf( "%s has started with rand_seed[%d], min_cycles[%d], max_cycles[%d].\n", client_name.c_str(), rand_seed, min_cycles, max_cycles ); 
    
  srand( rand_seed );

  rand_delta = max_cycles - min_cycles;

  // close the coordinator side channels
  __close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
  __close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
  __close( FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_COORDINATOR_TO_PREYCONTROLLER_WRITE_CHANNEL );
  __close( FD_COORDINATOR_TO_PREDCONTROLLER_WRITE_CHANNEL );
  __close( FD_COORDINATOR_TO_PREDPLANNER_WRITE_CHANNEL );

  if( strcmp( mythread->name, "prey-controller" ) == 0 ) {
    read_fd = FD_COORDINATOR_TO_PREYCONTROLLER_READ_CHANNEL;
    write_fd = FD_PREYCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL;

    // close the other channels that should not be accessed
    __close( FD_PREDCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
    __close( FD_COORDINATOR_TO_PREDCONTROLLER_READ_CHANNEL );
    __close( FD_PREDPLANNER_TO_COORDINATOR_WRITE_CHANNEL );
    __close( FD_COORDINATOR_TO_PREDPLANNER_READ_CHANNEL );
  } else if( strcmp( mythread->name, "pred-controller" ) == 0 ) {
    read_fd = FD_COORDINATOR_TO_PREDCONTROLLER_READ_CHANNEL;
    write_fd = FD_PREDCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL;

    // close the other channels that should not be accessed
    __close( FD_PREYCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
    __close( FD_COORDINATOR_TO_PREYCONTROLLER_READ_CHANNEL );
    __close( FD_PREDPLANNER_TO_COORDINATOR_WRITE_CHANNEL );
    __close( FD_COORDINATOR_TO_PREDPLANNER_READ_CHANNEL );
  } else if( strcmp( mythread->name, "pred-planner" ) == 0 ) {
    read_fd = FD_COORDINATOR_TO_PREDPLANNER_READ_CHANNEL;
    write_fd = FD_PREDPLANNER_TO_COORDINATOR_WRITE_CHANNEL;

    // close the other channels that should not be accessed
    __close( FD_PREYCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
    __close( FD_COORDINATOR_TO_PREYCONTROLLER_READ_CHANNEL );
    __close( FD_PREDCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
    __close( FD_COORDINATOR_TO_PREDCONTROLLER_READ_CHANNEL );
  } else {
    //printf( "ERROR: unable to reference correct program\nExiting\n" );
    return 1;
  }
  //printf( "%s write_fd:%d; read_fd:%d\n", client_name.c_str(), write_fd, read_fd );

  assert( rand_delta < RAND_MAX && rand_delta != 0 );

  // build the notification prototypes
  // - state notification -
  state_note.source = notification_t::CLIENT;
  state_note.type = notification_t::READ;
  state_note.pid = mythread->pid;

  // - command notification -
  command_note.source = notification_t::CLIENT;
  command_note.type = notification_t::WRITE;
  command_note.pid = mythread->pid;

  // - block notification -
  block_note.period = max_cycles;
  block_note.source = notification_t::CLIENT;
  block_note.type = notification_t::IDLE;
  block_note.pid = mythread->pid;

  return 0;
}

//-----------------------------------------------------------------------------
void shutdown( void ) {

  __close( read_fd );
  __close( write_fd );

  printf( "client[%s]: shutting down\n", mythread->name );
}

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] ) {

  if( init( argc, argv ) != 0 )
    //quit.store( 1, std::memory_order_seq_cst );
    quit = true;

  while( !quit ) {
  //while( !quit.load( std::memory_order_seq_cst ) ) {
    //request_state();
    compute_command();
    //publish_command();
    publish_block();
  }

  shutdown();

  return 0;
}
