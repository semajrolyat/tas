#include "log.h"
#include "os.h"
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
//-----------------------------------------------------------------------------
log_c::log_c( void ) {
  _defined = false;
  _allocated = false;
  _dirty = false;
  _capacity = 0;
  _cursor = 0;
  _buffer = NULL;
  _firstwrite = true;
  _create = false;
  _forward_to_stdout = false;
}

//-----------------------------------------------------------------------------
log_c::log_c( const char* path, const bool& create = false ) {
  _defined = true;
  _allocated = false;
  _dirty = false;
  _capacity = 0;
  _cursor = 0;
  _buffer = NULL;
  sprintf( _path, "%s", path);
  _firstwrite = true;
  _forward_to_stdout = false;

  _create = create;
}

//-----------------------------------------------------------------------------
log_c::~log_c( void) {
  if( _dirty ) flush();
  deallocate();
}

//-----------------------------------------------------------------------------
log_c::error_e log_c::allocate( const int& bytes ) {
  if( !_defined ) return ERROR_NOTDEFINED;
  if( _allocated ) return ERROR_REALLOC;

  _capacity = bytes;
  _cursor = 0;
  _buffer = (char*)malloc( sizeof(char) * _capacity );
  if( _buffer == NULL ) return ERROR_NOALLOC;

  _allocated = true;

  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
void log_c::deallocate( void ) {
  if( !_defined || !_allocated ) return;

  free( _buffer );
  _allocated = false;
  _capacity = 0;
  _cursor = 0;
}

//-----------------------------------------------------------------------------
log_c::error_e log_c::flush( void ) {
  // if not defined, return error
  if( !_defined ) return ERROR_NOTDEFINED;
  // if not allocated, return error
  if( !_allocated ) return ERROR_NOALLOC;
  // if not dirty then no need to write
  if( !_dirty ) return ERROR_NONE;

  int fd;
  // insert target
  if( _firstwrite ) {
    // open for overwriting
    fd = __open( _path, O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH );
    _firstwrite = false;
  } else {
    // open for appending
    fd = __open( _path, O_WRONLY | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH );
  }

  // validate the file descriptor returned from the open call
  if( fd == -1 ) return ERROR_OPEN;

  // write the buffer data to the file
///*
  if( __write( fd, _buffer, _cursor ) == -1 )
    return ERROR_WRITE;
//*/
  //while( __write(fd, _buffer, _cursor) == -1 );

  // close the file
  __close( fd );

  if( _forward_to_stdout ) {
    if( __write( 1, _buffer, _cursor ) == -1 )
      return ERROR_WRITE;    
  }

  // reset the cursor
  _cursor = 0;

  // reset the dirty flag
  _dirty = false;

  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
log_c::error_e log_c::write( const char* str ) {
  if( !_defined ) return ERROR_NOTDEFINED;
  if( !_allocated ) return ERROR_NOALLOC;

  // get the length of the string
  int bytes = strlen( str );

  // check for overflow on copy
  if( bytes + _cursor >= _capacity ) return ERROR_OVERFLOW;

  // copy the data
  //memcpy( _buffer + _cursor, str, bytes );
  for( unsigned i = 0; i < bytes; i++ ) {
    _buffer[i + _cursor] = str[i];
  }

  // update the cursor
  _cursor += bytes;

  // set the dirty flag
  _dirty = true;

  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
int log_c::capacity( void ) {
  return _capacity;
}

//-----------------------------------------------------------------------------
int log_c::size( void ) {
  return _cursor;
}

//-----------------------------------------------------------------------------
bool log_c::open( void ) {
  // if not defined, not open
  if( !_defined ) return false;
  // if not allocated, not open
  if( !_allocated ) return false;

  // otherwise, open
  return true;
}
//-----------------------------------------------------------------------------
