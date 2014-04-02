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

#include "os.h"

//-----------------------------------------------------------------------------
// Note: There is reimplementation here of fundamental tools (shared memory) 
// being used in client_message_buffer_c but there are some differences.  Ideal
// would be to abstract fundamental into templated shared_memory class, but
// the differences make it more complicated, for future work, find the median
// ground and implement shared_memory_c, but for now, note that there is 
// reimplementation here.
//-----------------------------------------------------------------------------
#include <pthread.h>
#include <string>
//-----------------------------------------------------------------------------

class log_c {
private:
  bool _create;
  bool _allocated;
  bool _defined;
  bool _dirty;

  int _capacity;
  int _cursor;
  std::string _path;
  bool _firstwrite;

  char* _buffer;
  mutex_c _mutex;

/*
  // note: these are part of shared basis with client_message_buffer_c
  std::string _buffer_name;
  int _fd_buffer;
  char* _buffer;
 
  std::string _mutex_name;
  int _fd_mutex;
  pthread_mutex_t* _mutex; 
*/
public:
  enum error_e {
    ERROR_NONE,           ///< No error, operation was successful.
    ERROR_NOTDEFINED,     ///< The log was not defined before using.
    ERROR_NOALLOC,        ///< Allocation failed or the buffer is not allocated.
    ERROR_REALLOC,        ///< Attempt to reallocate a buffer that is allocated.
    ERROR_OVERFLOW,       ///< Insert would exceed buffer size.
    ERROR_OPEN,           ///< Flush could not open the file specified.
    ERROR_WRITE,          ///< Flush could not write to the file specified.
// note : these are part of shared basis with client_message_buffer_c
    ERROR_MAPPING,
    ERROR_UNMAPPING,
//    ERROR_OPENING,
    ERROR_TRUNCATING,
    ERROR_UNLINKING,
    ERROR_SYNCING,
    ERROR_MUTEX
  };

  log_c( void );
  log_c( const std::string& path, const bool& create );
  //log_c( const std::string& path, const char* buffer_name, const char* mutex_name, const bool& create );
  virtual ~log_c( void );

  error_e allocate( const int& size );
  void deallocate( void );
  error_e flush( void );
  error_e write( const std::string& str );

  int capacity( void );
  int size( void );
  bool open( void );
/*
private:
  // note : these are part of shared basis with client_message_buffer_c
  error_e init_mutex( void );
  error_e delete_mutex( void );
  error_e init_buffer( void );
  error_e delete_buffer( void );

public:
  error_e open( void );
  void close( void );
  error_e write( const char* msg );
  //error_e read( char* msg )
*/
};

//-----------------------------------------------------------------------------

#endif // _LOG_H_
