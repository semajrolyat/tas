#include "os.h"

//-----------------------------------------------------------------------------

//#ifndef SYS_CLOSE_FD
//#define SYS_CLOSE_FD
/// Aliasing the system close(2) function.  Necessary because the buffer has
/// a close method and the compiler doesn't want to use the system function
/// because they have the same name but different signatures
void sys_close_fd( const int& fd ) {
    close( fd );
}
//#endif

//-----------------------------------------------------------------------------
ssize_t sys_write_fd( int fd, const void* buf, size_t count ) {
  return write( fd, buf, count );
}

//-----------------------------------------------------------------------------
ssize_t sys_read_fd( int fd, void* buf, size_t count ) {
  return read( fd, buf, count );
}


