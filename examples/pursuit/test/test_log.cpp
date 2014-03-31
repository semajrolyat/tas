#include "../log.h"

#include <string>
#include <stdio.h>
#include <stdlib.h>

#define LOG_CAPACITY 64

//-----------------------------------------------------------------------------
void test1( void ) {
  log_c log;
  log_c::error_e err;
  std::string data, err_info;

  log = log_c( "test.log" );

  err = log.allocate( LOG_CAPACITY );
  if( err != log_c::ERROR_NONE ) {
    switch( err ) {
    case log_c::ERROR_NOINIT:
      err_info = "ERROR_NOINIT";
      break;
    case log_c::ERROR_REALLOC:
      err_info = "ERROR_REALLOC";
      break;
    case log_c::ERROR_NOALLOC:
      err_info = "ERROR_NOALLOC";
      break;
    default:
      err_info = "?";
      break; 
    }
    printf( "ERROR: failed to allocate the log. Info: %s\nExiting.\n", err_info.c_str() );
    exit( 1 );
  }

  data = "First Line.\nSecond Line.\n";
  err = log.write( data );
  if( err != log_c::ERROR_NONE ) {
    printf( "ERROR: failed to write to the log on first write.\nExiting.\n" );
    exit( 1 );
  }
  err = log.flush();
  if( err != log_c::ERROR_NONE ) {
    printf( "ERROR: failed to flush the log on first flush.\nExiting.\n" );
    exit( 1 );
  }

  data = "Third Line.\nFourth Line.\n";
  err = log.write( data );
  if( err != log_c::ERROR_NONE ) {
    printf( "ERROR: failed to write to the log on second write.\nExiting.\n" );
    exit( 1 );
  }
  err = log.flush();
  if( err != log_c::ERROR_NONE ) {
    printf( "ERROR: failed to flush the log on second flush.\nExiting.\n" );
    exit( 1 );
  }
}

//-----------------------------------------------------------------------------
int main( void ) {

  log_c log;
  log_c::error_e err;
  std::string data, err_info;

  log = log_c( "test.log" );
  err = log.allocate( LOG_CAPACITY );

  if( err != log_c::ERROR_NONE ) {
    switch( err ) {
    case log_c::ERROR_NOINIT:
      err_info = "ERROR_NOINIT";
      break;
    case log_c::ERROR_REALLOC:
      err_info = "ERROR_REALLOC";
      break;
    case log_c::ERROR_NOALLOC:
      err_info = "ERROR_NOALLOC";
      break;
    default:
      err_info = "?";
      break; 
    }
    printf( "ERROR: failed to allocate the log. Info: %s\nExiting.\n", err_info.c_str() );
    exit( 1 );
  }

  for( unsigned i = 1; i <= LOG_CAPACITY * 2; i++ ) {
    data = (i % 10) + 48;
    err = log.write( data );
    if( err != log_c::ERROR_NONE ) {
      printf( "ERROR: failed to write to the log on first write.\nExiting.\n" );
      exit( 1 );
    }
    if( log.size() >= log.capacity() / 2 ) {
      data = "\n";
      err = log.write( data );
      if( err != log_c::ERROR_NONE ) {
        printf( "ERROR: failed to write to the log on first write.\nExiting.\n" );
        exit( 1 );
      }     
      err = log.flush();
      if( err != log_c::ERROR_NONE ) {
        printf( "ERROR: failed to flush the log on first write.\nExiting.\n" );
        exit( 1 );
      }
    }
  }
  return 0;
}
