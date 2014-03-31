#ifndef _LOG_H_
#define _LOG_H_

//-----------------------------------------------------------------------------

#include <assert.h>
#include <string>
#include <string.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

//-----------------------------------------------------------------------------
class log_c {
private:
  bool _allocated;
  bool _initialized;
  bool _dirty;
  int _capacity;
  char* _buffer;
  int _cursor;
  std::string _path;
  bool _firstwrite;

public:
  enum error_e {
    ERROR_NONE,           ///< No error, operation was successful.
    ERROR_NOINIT,         ///< The log was not initialized before using.
    ERROR_NOALLOC,        ///< Allocation failed or the buffer is not allocated.
    ERROR_REALLOC,        ///< Attempt to reallocate a buffer that is allocated.
    ERROR_OVERFLOW,       ///< Insert would exceed buffer size.
    ERROR_OPEN,           ///< Flush could not open the file specified.
    ERROR_WRITE           ///< Flush could not write to the file specified.
  };

  log_c( void );
  log_c( const std::string& path );
  virtual ~log_c( void );

  error_e allocate( const int& size );
  void deallocate( void );
  error_e flush( void );
  error_e write( const std::string& str );

  int capacity( void );
  int size( void );
  bool open( void );
};

//-----------------------------------------------------------------------------

#endif // _LOG_H_
