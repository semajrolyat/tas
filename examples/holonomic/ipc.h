#ifndef _IPC_H_
#define _IPC_H_

#include "os.h"
#include "error.h"

//-----------------------------------------------------------------------------
// MAPPED IPC CHANNELS
//-----------------------------------------------------------------------------

#define FD_ERROR_LOG				    100

#define FD_COORDINATOR_TO_PREY_READ_CHANNEL   1000
#define FD_COORDINATOR_TO_PREY_WRITE_CHANNEL  1001

#define FD_COORDINATOR_TO_PREDATOR_READ_CHANNEL   1002
#define FD_COORDINATOR_TO_PREDATOR_WRITE_CHANNEL  1003

#define FD_COORDINATOR_TO_PLANNER_READ_CHANNEL   1004
#define FD_COORDINATOR_TO_PLANNER_WRITE_CHANNEL  1005

#define FD_PREY_TO_COORDINATOR_READ_CHANNEL   1006
#define FD_PREY_TO_COORDINATOR_WRITE_CHANNEL  1007

#define FD_PREDATOR_TO_COORDINATOR_READ_CHANNEL   1008
#define FD_PREDATOR_TO_COORDINATOR_WRITE_CHANNEL  1009

#define FD_PLANNER_TO_COORDINATOR_READ_CHANNEL   1010
#define FD_PLANNER_TO_COORDINATOR_WRITE_CHANNEL  1011

#define FD_TIMER_TO_COORDINATOR_READ_CHANNEL   1012
#define FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL  1013

#define FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL   1014
#define FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL  1015

//-----------------------------------------------------------------------------
enum notification_type_e {
  NOTIFICATION_UNDEFINED = 0,
  NOTIFICATION_IDLE,
  NOTIFICATION_REQUEST,
  NOTIFICATION_RESPONSE,
  NOTIFICATION_TIMER,
  NOTIFICATION_WAKEUP
};

//-----------------------------------------------------------------------------
class notification_c {
public:
    notification_c( void ) { }
    virtual ~notification_c( void ) { }

    unsigned type;
    timestamp_t ts;
    
};

//-----------------------------------------------------------------------------
class pipe_c {
public:
  enum pipe_error_e {
    PIPE_ERROR_NONE = 0,
    PIPE_ERROR_SYS_PIPE,
    PIPE_ERROR_DUPE_READ_CHANNEL,
    PIPE_ERROR_DUPE_WRITE_CHANNEL,
    PIPE_ERROR_READ,
    PIPE_ERROR_WRITE
  };

  pipe_c( void ) {
    block_on_write = false;
  }

  pipe_c( const int& _fd_read_channel, const int& _fd_write_channel, const bool& _block_on_write = false ) {
    fd_read_channel = _fd_read_channel;
    fd_write_channel = _fd_write_channel;
    block_on_write = _block_on_write;
  }

  virtual ~pipe_c( void ) {}

  int fd_read_channel;
  int fd_write_channel;

  bool block_on_write;	    // effectively read only after open. not enforced

  pipe_error_e open(  ) {
    int flags;
    int fd[2];

    // NOTE: read channel needs to be blocking to force block waiting for event
    // NOTE: write channel is non-blocking (won't ever fill up buffer anyway)

    // open the timer to coordinator channel for timer events
    if( pipe( fd ) != 0 ) 
      return PIPE_ERROR_SYS_PIPE;

    // set flags of the open pipe    
    flags = fcntl( fd[0], F_GETFL, 0 );
    fcntl( fd[0], F_SETFL, flags );
    flags = fcntl( fd[1], F_GETFL, 0 );
    if( block_on_write )
      fcntl( fd[1], F_SETFL, flags );
    else
      fcntl( fd[1], F_SETFL, flags | O_NONBLOCK );

    // duplicate the read channel into the prescribed identifier
    if( dup2( fd[0], fd_read_channel ) == -1 ) {
      sys_close_fd( fd[0] );
      sys_close_fd( fd[1] );
      return PIPE_ERROR_DUPE_READ_CHANNEL;
    }

    // duplicate the write channel into the prescribed identifier
    if( dup2( fd[1], fd_write_channel ) == -1 ) {
      sys_close_fd( fd_read_channel );
      sys_close_fd( fd[1] );
      return PIPE_ERROR_DUPE_WRITE_CHANNEL;
    }
    return PIPE_ERROR_NONE;
  }

  void close( void ) {
    sys_close_fd( fd_read_channel );
    sys_close_fd( fd_write_channel );
  }

  pipe_error_e write( const notification_c& note ) {
    if( sys_write_fd( fd_write_channel, &note, sizeof( notification_c ) ) == -1 ) 
       return PIPE_ERROR_WRITE;
    return PIPE_ERROR_NONE;
  }

  pipe_error_e read( notification_c& note ) {
    if( sys_read_fd( fd_read_channel, &note, sizeof( notification_c ) ) == -1 ) 
      return PIPE_ERROR_READ;
    return PIPE_ERROR_NONE;
  }

  static void log_error( log_c& log, const pipe_error_e& error, std::string src, std::string caller_method, std::string called_method ) {
    char errstr[256];

    std::string err;
    switch( error ) {
    default:
    case PIPE_ERROR_NONE: 
      err = "UNDEFINED";
      break;
    case PIPE_ERROR_SYS_PIPE:
      err = "PIPE_ERROR_SYS_PIPE";
      break;
    case PIPE_ERROR_DUPE_READ_CHANNEL:
      err = "PIPE_ERROR_DUPE_READ_CHANNEL";
      break;
    case PIPE_ERROR_DUPE_WRITE_CHANNEL:
      err = "PIPE_ERROR_DUPE_WRITE_CHANNEL";
      break;
    case PIPE_ERROR_READ:
      err = "PIPE_ERROR_READ";
      break;
    case PIPE_ERROR_WRITE:
      err = "PIPE_ERROR_WRITE";
      break;
    }
    sprintf( errstr, "(%s) %s failed calling %s : %s\n", src.c_str(), caller_method.c_str(), called_method.c_str(), err.c_str() );

    log.write( errstr );
  }
};

/*
//-----------------------------------------------------------------------------
class channel_c {
public:
    channel_c( void ) { }
    virtual ~channel_c( void ) { }

    unsigned read;
    unsigned write;

};

//-----------------------------------------------------------------------------
class event_c {  
public:
    event_c( void ) { }
    virtual ~event_c( void ) { }

};
*/
//-----------------------------------------------------------------------------

#endif // _IPC_H_
