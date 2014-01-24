/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

memory.h
-----------------------------------------------------------------------------*/

#ifndef _MEMORY_H_
#define _MEMORY_H_

//-----------------------------------------------------------------------------

#include <string.h>
#include <iostream>
#include <pthread.h>
#include <assert.h>

#include "message.h"
#include "os.h"

//-----------------------------------------------------------------------------

enum buffer_err_e {
  BUFFER_ERR_NONE = 0,
  BUFFER_ERR_MAPPING,
  BUFFER_ERR_UNMAPPING,
  BUFFER_ERR_OPENING,
  BUFFER_ERR_TRUNCATING,
  BUFFER_ERR_UNLINKING,
  BUFFER_ERR_SYNCING,
  BUFFER_ERR_MUTEX
};

//-----------------------------------------------------------------------------
/*
#ifndef _MESSAGE_BUFFER_ID
#define _MESSAGE_BUFFER_ID
//#define ACTUATOR_MESSAGE_BUFFERS_MAX 2
const char* BUFFER_NAME = "/amsgbuffer";
#endif
*/
//-----------------------------------------------------------------------------

class sharedbuffer_c {
private:

  bool                create;
  bool                initialized;
  bool                opened;

  std::string         buffer_name;
  std::string         mutex_name;

  int                 fd_mutex;
  pthread_mutex_t*    mutex;

  int                 fd_buffer;
  act_msg_c* 	        buffer;

public:

  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
  /// Default Constructor
  sharedbuffer_c( void ) {
    initialized = false;
    opened = false;
    create = true;
    mutex = NULL;
    buffer = NULL;
  }

  //---------------------------------------------------------------------------
  sharedbuffer_c( std::string _buffer_name, const bool& _create ) {
    buffer_name = _buffer_name;
    mutex_name = _buffer_name + std::string( "_mutex" );
    create = _create;
    opened = false;
    initialized = true;
  }

  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------

  virtual ~sharedbuffer_c( void ) {
    if( opened ) close( );
  }

private:
  //---------------------------------------------------------------------------
  buffer_err_e init_mutex( void ) {

    void* shm_mutex_addr;
    pthread_mutexattr_t mutexattr;

    if( create )
      fd_mutex = shm_open( mutex_name.c_str( ), O_CREAT | O_RDWR, S_IRUSR | S_IWUSR );
    else
      fd_mutex = shm_open( mutex_name.c_str( ), O_RDWR, S_IRUSR | S_IWUSR );

    if( fd_mutex == -1 ) {
      perror( "shm_open()" );
      return BUFFER_ERR_OPENING;
    }

    if( ftruncate( fd_mutex, sizeof(pthread_mutex_t) ) == -1 ) {
      return BUFFER_ERR_TRUNCATING;
    }


    shm_mutex_addr = mmap( NULL, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd_mutex, 0 );

    if( shm_mutex_addr == MAP_FAILED ) {
      perror( "mmap()" );
      sys_close_fd( fd_mutex );
      return BUFFER_ERR_MAPPING;
    }

    mutex = static_cast<pthread_mutex_t*> ( shm_mutex_addr );
    if( create ) {
      pthread_mutexattr_init( &mutexattr );

      pthread_mutexattr_setpshared( &mutexattr, PTHREAD_PROCESS_SHARED );

      if( pthread_mutex_init( mutex, &mutexattr ) != 0 ) {
        // what other handling should be done in response.  Close the fd?
        return BUFFER_ERR_MUTEX;
      }
      pthread_mutexattr_destroy( &mutexattr );

      if( msync( shm_mutex_addr, sizeof(pthread_mutex_t), MS_SYNC | MS_INVALIDATE ) != 0 ) {
        // what other handling should be done in response.  Close the fd, release the mutex?
        perror( "msync()" );
        return BUFFER_ERR_SYNCING;
      }
    }
    return BUFFER_ERR_NONE;
  }

  //---------------------------------------------------------------------------
  buffer_err_e delete_mutex( void ) {

    sys_close_fd( fd_mutex );

    if( create ) {
      if( munmap( (void*)mutex, sizeof(pthread_mutex_t) ) ) {
        perror( "munmap()" );
        return BUFFER_ERR_UNMAPPING;
      }

      if( shm_unlink( mutex_name.c_str( ) ) != 0 ) {
        return BUFFER_ERR_UNLINKING;
      }
    }
    return BUFFER_ERR_NONE;
  }

  //---------------------------------------------------------------------------
  buffer_err_e init_buffer( void ) {

    void* shm_buffer_addr;

    if( create )
      fd_buffer = shm_open( buffer_name.c_str( ), O_CREAT | O_RDWR, S_IRUSR | S_IWUSR );
    else
      fd_buffer = shm_open( buffer_name.c_str( ), O_RDWR, S_IRUSR | S_IWUSR );

    if( fd_buffer == -1 ) {
      perror( "shm_open()" );
      return BUFFER_ERR_OPENING;
    }
    if( ftruncate( fd_buffer, sizeof(act_msg_c) ) == -1 ) {
      return BUFFER_ERR_TRUNCATING;
    }

    shm_buffer_addr = mmap( NULL, sizeof(act_msg_c), PROT_READ | PROT_WRITE, MAP_SHARED, fd_buffer, 0 );

    if( shm_buffer_addr == MAP_FAILED ) {
      perror( "mmap()" );
      sys_close_fd( fd_mutex );
      return BUFFER_ERR_MAPPING;
    }

    buffer = static_cast<act_msg_c*> ( shm_buffer_addr );

    return BUFFER_ERR_NONE;
  }

  //---------------------------------------------------------------------------
  buffer_err_e delete_buffer( void ) {

    sys_close_fd( fd_buffer );

    if( create ) {
      if( munmap( (void*)buffer, sizeof( act_msg_c ) ) ) {
        perror("munmap()");
        return BUFFER_ERR_UNMAPPING;
      }

      if( shm_unlink( buffer_name.c_str( ) ) != 0 ) {
        return BUFFER_ERR_UNLINKING;
      }
    }
    return BUFFER_ERR_NONE;
  }

  //---------------------------------------------------------------------------

public:

  //---------------------------------------------------------------------------
  buffer_err_e open( void ) {
    assert( initialized && !opened );

    buffer_err_e result;
    result = init_mutex( );
    if( result != BUFFER_ERR_NONE ) {
      // major problem.  No option but to bomb out.
      return result;
    }
    result = init_buffer( );
    if( result != BUFFER_ERR_NONE ) {
      // major problem.  No option but to bomb out.
      return result;
    }

    opened = true;

    return BUFFER_ERR_NONE;
  }

  //---------------------------------------------------------------------------
  void close( void ) {

    delete_buffer( );
    delete_mutex( );

    opened = false;

    // TODO : better clean up of resources to prevent leak/artifacts
  }

  //---------------------------------------------------------------------------
  buffer_err_e write( const act_msg_c& msg ) {
    assert( opened );

    pthread_mutex_lock( mutex );

    memcpy( &buffer->body, &msg.body, sizeof(struct act_msg_body_t) );
    memcpy( &buffer->header, &msg.header, sizeof(struct act_msg_header_t) );

    if( msync( (void*)buffer, sizeof( act_msg_c ), MS_SYNC ) != 0 ) {
      perror( "msync()" );
      return BUFFER_ERR_SYNCING;
    }

    pthread_mutex_unlock( mutex );

    return BUFFER_ERR_NONE;
  }

  //---------------------------------------------------------------------------
  buffer_err_e read( act_msg_c& msg ) {
    assert( opened );

    pthread_mutex_lock( mutex );

    memcpy( &msg.header, &buffer->header, sizeof(struct act_msg_header_t) );
    memcpy( &msg.body, &buffer->body, sizeof(struct act_msg_body_t) );

    pthread_mutex_unlock( mutex );

    return BUFFER_ERR_NONE;
  }

  //---------------------------------------------------------------------------

};

//-----------------------------------------------------------------------------

#endif // _MEMORY_H_
