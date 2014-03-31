#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string>

#include "../time.h"
#include "../os.h"
#include "../notification.h"

#include "../channels.h"

//-----------------------------------------------------------------------------

volatile unsigned int index;
#include <errno.h>
//#include <unistd.h>
//#include <sys/select.h>
//#include <sys/types.h>

int main( int argc, char* argv[] ) {
  std::string client_name = argv[0];
  std::string err;
  notification_t note;

  if( argc < 4 ) {
    //printf( "ERROR: client process[%s] was not given enough arguments\nExiting client process.\n", client_name.c_str() );
    exit( 1 );
  }
  unsigned rand_seed = (unsigned) atoi( argv[1] );
  unsigned min_cycles = (unsigned) atoi( argv[2] );
  unsigned max_cycles = (unsigned) atoi( argv[3] );

  //printf( "%s has started with rand_seed[%d], min_cycles[%d], max_cycles[%d].\n", client_name.c_str(), rand_seed, min_cycles, max_cycles ); 
    
  pid_t pid = getpid();

  srand( rand_seed );

  unsigned rand_delta = max_cycles - min_cycles;

  note.blocktill = 0;
  note.source = notification_t::CLIENT;
  note.type = notification_t::IDLE;
  note.pid = pid;
  note.ts = generate_timestamp();
/*
  if( __write( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL, &note, sizeof(notification_t) ) == -1 ) {
    err = "(" + client_name + ") failed making system call write(...)";
    printf( "ERROR: %s\n", err.c_str() );
  }
*/
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

  int write_fd, read_fd;
  if( client_name.compare( "prey-controller" ) == 0 ) {
    read_fd = FD_COORDINATOR_TO_PREYCONTROLLER_READ_CHANNEL;
    write_fd = FD_PREYCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL;

    // close the other channels that should not be accessed
    __close( FD_PREDCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
    __close( FD_COORDINATOR_TO_PREDCONTROLLER_READ_CHANNEL );
    __close( FD_PREDPLANNER_TO_COORDINATOR_WRITE_CHANNEL );
    __close( FD_COORDINATOR_TO_PREDPLANNER_READ_CHANNEL );
  } else if( client_name.compare( "pred-controller" ) == 0 ) {
    read_fd = FD_COORDINATOR_TO_PREDCONTROLLER_READ_CHANNEL;
    write_fd = FD_PREDCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL;

    // close the other channels that should not be accessed
    __close( FD_PREYCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
    __close( FD_COORDINATOR_TO_PREYCONTROLLER_READ_CHANNEL );
    __close( FD_PREDPLANNER_TO_COORDINATOR_WRITE_CHANNEL );
    __close( FD_COORDINATOR_TO_PREDPLANNER_READ_CHANNEL );
  } else if( client_name.compare( "pred-planner" ) == 0 ) {
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

  note.blocktill = 0;
  note.source = notification_t::CLIENT;
  note.type = notification_t::IDLE;
  note.pid = pid;

  std::string eno;

  while( true ) {
    index = 0;
    unsigned cycles_to_run = (rand() % rand_delta) + min_cycles;
    //printf( "%s generated cycles_to_run: %u\n", client_name.c_str(), cycles_to_run );
    for( unsigned i = 0; i < cycles_to_run; i++ ) {
      index++;
    }

    note.ts = generate_timestamp();
    note.blocktill += max_cycles;

    if( __write( write_fd, &note, sizeof(notification_t) ) == -1 ) {
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
      err = "(" + client_name + ") failed making system call write(...)" + eno;
      printf( "ERROR: %s\n", err.c_str() );
    }
  }

  __close( read_fd );
  __close( write_fd );
  return 0;
}
