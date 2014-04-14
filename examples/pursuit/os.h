#ifndef _OS_H_
#define _OS_H_

#include <string>
#include <vector>

#include <sys/types.h>  // POSIX
#include <pthread.h>    // POSIX

//-----------------------------------------------------------------------------

enum os_error_e {
  OS_ERROR_NONE,
  OS_ERROR_AGAIN,
  OS_ERROR_BADF,
  OS_ERROR_DESTADDRREQ,
  OS_ERROR_DQUOT,
  OS_ERROR_FAULT,
  OS_ERROR_FBIG,
  OS_ERROR_INVAL,
  OS_ERROR_INTR,
  OS_ERROR_IO,
  OS_ERROR_NOSPC,
  OS_ERROR_PIPE,
  OS_ERROR_OTHER  // catch all for unexpected errno's
};

//-----------------------------------------------------------------------------

void __select( const std::vector<int>& fds, fd_set& channels );
bool __isset( const int& fd, fd_set& channels );
int __open( const char* path, int oflags, const mode_t& mode );
void __close( const int& fd );
ssize_t __read( int fd, void* buffer, size_t bytes );
os_error_e __write( int fd, void* buffer, size_t bytes_to_write, ssize_t& bytes_written );

//-----------------------------------------------------------------------------

class mutex_c {
private:
  bool                _create;
  bool                _defined;
  bool                _open;

  std::string         _name;
  int                 _fd;
  pthread_mutex_t*    _mutex;
public:
  enum error_e {
    ERROR_NONE,
    ERROR_OPEN,
    ERROR_TRUNCATE,
    ERROR_MAP,
    ERROR_UNMAP,
    ERROR_INIT,
    ERROR_SYNC,
    ERROR_UNLINK
  };

  mutex_c( void );
  mutex_c( const char* name, const bool& create );
  virtual ~mutex_c( void );

  error_e init( void );
  error_e destroy( void );
  void lock( void );
  void unlock( void );

  bool open( void ) const;
  bool defined( void ) const;
  bool creator( void ) const;
  std::string name( void ) const;
};

//-----------------------------------------------------------------------------

#endif
