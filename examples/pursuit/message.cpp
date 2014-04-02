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

//-----------------------------------------------------------------------------
client_message_buffer_c::client_message_buffer_c( void ) {
  _defined = false;
  _create = false;
  _open = false;
  _buffer = NULL;
}

//-----------------------------------------------------------------------------
client_message_buffer_c::client_message_buffer_c( const char* buffer_name, const char* mutex_name, const bool& create = false ) {
  assert( buffer_name != NULL && mutex_name != NULL );

  _buffer_name = buffer_name;
  _create = create;
  _open = false;
  _defined = true;

  _mutex = mutex_c( mutex_name, create );
}

//-----------------------------------------------------------------------------
client_message_buffer_c::~client_message_buffer_c( void ) {
  if( _open ) close();
}

//-----------------------------------------------------------------------------
client_message_buffer_c::buffer_error_e client_message_buffer_c::init( void ) {

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
    return ERROR_MAPPING;
  }

  _buffer = static_cast<client_message_t*> ( shm_buffer_addr );

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
client_message_buffer_c::buffer_error_e client_message_buffer_c::destroy( void ) {
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
  assert( _defined && !_open );

  buffer_error_e result;
  mutex_c::error_e mutex_err;
  mutex_err = _mutex.init( );
  if( mutex_err != mutex_c::ERROR_NONE ) {
    // major problem.  No option but to bomb out.
    return ERROR_MUTEX;
  }
  result = init( );
  if( result != ERROR_NONE ) {
    // major problem.  No option but to bomb out.
    _mutex.destroy();
    return result;
  }

  _open = true;

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
void client_message_buffer_c::close( void ) {
  destroy( );
  _mutex.destroy();

  _open = false;

  // TODO : better clean up of resources to prevent leak/artfacts
}

//-----------------------------------------------------------------------------
client_message_buffer_c::buffer_error_e client_message_buffer_c::write( const client_message_t& m ) {
  assert( _open );

  _mutex.lock();

  memcpy( &_buffer->header, &m.header, sizeof(struct client_message_header_t) );
  memcpy( &_buffer->prey_state, &m.prey_state, sizeof(struct state_t) );
  memcpy( &_buffer->pred_state, &m.pred_state, sizeof(struct state_t) );
  memcpy( &_buffer->pred_control, &m.pred_control, sizeof(struct control_t) );

  if( msync( (void*)_buffer, sizeof(client_message_t), MS_SYNC ) != 0 ) {
    perror( "msync()" );
    return ERROR_SYNCING;
  }

  _mutex.unlock();

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
client_message_buffer_c::buffer_error_e client_message_buffer_c::read( client_message_t& m ) {
  assert( _open );

  _mutex.lock();

  memcpy( &m.header, &_buffer->header, sizeof(struct client_message_header_t) );
  memcpy( &m.prey_state, &_buffer->prey_state, sizeof(struct state_t) );
  memcpy( &m.pred_state, &_buffer->pred_state, sizeof(struct state_t) );
  memcpy( &m.pred_control, &_buffer->pred_control, sizeof(struct control_t) );

  _mutex.unlock();

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
