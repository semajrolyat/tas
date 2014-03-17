#include "message.h"

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <assert.h>
#include <stdio.h>

#include "os.h"

//-----------------------------------------------------------------------------
client_message_buffer_c::client_message_buffer_c( void ) {
  _initialized = false;
  _create = false;
  _open = false;
  _mutex = NULL;
  _buffer = NULL;
}

//-----------------------------------------------------------------------------
client_message_buffer_c::client_message_buffer_c( const char* buffer_name, const char* mutex_name, const bool& create = false ) {
  assert( buffer_name != NULL && mutex_name != NULL );

  _buffer_name = buffer_name;
  _mutex_name = mutex_name;
  _create = create;
  _open = false;
  _initialized = true;
}

//-----------------------------------------------------------------------------
client_message_buffer_c::~client_message_buffer_c( void ) {
  if( _open ) close();
}

//-----------------------------------------------------------------------------
client_message_buffer_c::buffer_error_e client_message_buffer_c::init_mutex( void ) {
  void* shm_mutex_addr;
  pthread_mutexattr_t mutexattr;

  if( _create )
    _fd_mutex = shm_open( _mutex_name.c_str( ), O_CREAT | O_RDWR, S_IRUSR | S_IWUSR );
  else
    _fd_mutex = shm_open( _mutex_name.c_str( ), O_RDWR, S_IRUSR | S_IWUSR );

  if( _fd_mutex == -1 ) {
    perror( "shm_open()" );
    return ERROR_OPENING;
  }

  if( ftruncate( _fd_mutex, sizeof(pthread_mutex_t) ) == -1 ) {
    return ERROR_TRUNCATING;
  }

  shm_mutex_addr = mmap( NULL, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED, _fd_mutex, 0 );

  if( shm_mutex_addr == MAP_FAILED ) {
    perror( "mmap()" );
    __close( _fd_mutex );
    return ERROR_MAPPING;
  }

  _mutex = static_cast<pthread_mutex_t*> ( shm_mutex_addr );
  if( _create ) {
    pthread_mutexattr_init( &mutexattr );
    pthread_mutexattr_setpshared( &mutexattr, PTHREAD_PROCESS_SHARED );

    if( pthread_mutex_init( _mutex, &mutexattr ) != 0 ) {
      // what other handling should be done in response.  Close the fd?
      return ERROR_MUTEX;
    }
    pthread_mutexattr_destroy( &mutexattr );

    if( msync( shm_mutex_addr, sizeof(pthread_mutex_t), MS_SYNC | MS_INVALIDATE ) != 0 ) {
      // what other handling should be done in response.  Close the fd, release the mutex?
      perror( "msync()" );
      return ERROR_SYNCING;
    }
  }
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
client_message_buffer_c::buffer_error_e client_message_buffer_c::delete_mutex( void ) {

  __close( _fd_mutex );

  if( _create ) {
    if( munmap( (void*)_mutex, sizeof(pthread_mutex_t) ) ) {
      perror( "munmap()" );
      return ERROR_UNMAPPING;
    }
    if( shm_unlink( _mutex_name.c_str( ) ) != 0 ) {
      return ERROR_UNLINKING;
    }
  }
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
client_message_buffer_c::buffer_error_e client_message_buffer_c::init_buffer( void ) {

  void* shm_buffer_addr;

  if( _create )
    _fd_buffer = shm_open( _buffer_name.c_str( ), O_CREAT | O_RDWR, S_IRUSR | S_IWUSR );
  else
    _fd_buffer = shm_open( _buffer_name.c_str( ), O_RDWR, S_IRUSR | S_IWUSR );

  if( _fd_buffer == -1 ) {
    perror( "shm_open()" );
    return ERROR_OPENING;
  }

  if( ftruncate( _fd_buffer, sizeof(client_message_t) ) == -1 ) {
    return ERROR_TRUNCATING;
  }

  shm_buffer_addr = mmap( NULL, sizeof(client_message_t), PROT_READ | PROT_WRITE, MAP_SHARED, _fd_buffer, 0 );

  if( shm_buffer_addr == MAP_FAILED ) {
    perror( "mmap()" );
    __close( _fd_mutex );
    return ERROR_MAPPING;
  }

  _buffer = static_cast<client_message_t*> ( shm_buffer_addr );

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
client_message_buffer_c::buffer_error_e client_message_buffer_c::delete_buffer( void ) {
  __close( _fd_buffer );

  if( _create ) {
    if( munmap( (void*)_buffer, sizeof(client_message_t) ) ) {
       perror("munmap()");
       return ERROR_UNMAPPING;
    }

    if( shm_unlink( _buffer_name.c_str( ) ) != 0 ) {
      return ERROR_UNLINKING;
    }
  }

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
client_message_buffer_c::buffer_error_e client_message_buffer_c::open( void ) {
  assert( _initialized && !_open );

  buffer_error_e result;
  result = init_mutex( );
  if( result != ERROR_NONE ) {
    // major problem.  No option but to bomb out.
    return result;
  }
  result = init_buffer( );
  if( result != ERROR_NONE ) {
    // major problem.  No option but to bomb out.
    return result;
  }

  _open = true;

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
void client_message_buffer_c::close( void ) {
  delete_buffer( );
  delete_mutex( );

  _open = false;

  // TODO : better clean up of resources to prevent leak/artfacts
}

//-----------------------------------------------------------------------------
client_message_buffer_c::buffer_error_e client_message_buffer_c::write( const client_message_t& m ) {
  assert( _open );

  pthread_mutex_lock( _mutex );

  memcpy( &_buffer->header, &m.header, sizeof(struct client_message_header_t) );
  memcpy( &_buffer->prey_state, &m.prey_state, sizeof(struct state_t) );
  memcpy( &_buffer->pred_state, &m.pred_state, sizeof(struct state_t) );
  memcpy( &_buffer->pred_control, &m.pred_control, sizeof(struct control_t) );

  if( msync( (void*)_buffer, sizeof(client_message_t), MS_SYNC ) != 0 ) {
    perror( "msync()" );
    return ERROR_SYNCING;
  }

  pthread_mutex_unlock( _mutex );

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
client_message_buffer_c::buffer_error_e client_message_buffer_c::read( client_message_t& m ) {
  assert( _open );

  pthread_mutex_lock( _mutex );

  memcpy( &m.header, &_buffer->header, sizeof(struct client_message_header_t) );
  memcpy( &m.prey_state, &_buffer->prey_state, sizeof(struct state_t) );
  memcpy( &m.pred_state, &_buffer->pred_state, sizeof(struct state_t) );
  memcpy( &m.pred_control, &_buffer->pred_control, sizeof(struct control_t) );

  pthread_mutex_unlock( _mutex );

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
