#include "os.h"

#include <vector>
#include <assert.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
//-----------------------------------------------------------------------------
//#include <stdio.h>
void __select( const std::vector<int>& fds, fd_set& channels ) {
  int max_fd = 0;

  assert( fds.size() > 0 );

  FD_ZERO( &channels );
  for( int i = 0; i < fds.size(); i++ ) {
    max_fd = std::max( max_fd, fds[i] );
    FD_SET( fds[i], &channels );
    //printf( "fd:%d, ", fds[i] );
  }
  //printf( "max_fd:%d\n", max_fd );
  select( max_fd + 1, &channels, NULL, NULL, NULL );

/*if( select( max_fd + 1, &channels, NULL, NULL, NULL ) == -1 ) {
    errno{ EBADF, EINTR, EINVAL, ENOMEM }
  }*/
}

//-----------------------------------------------------------------------------
bool __isset( const int& fd, fd_set& channels ) {
  if( FD_ISSET( fd, &channels ) ) return true;
  return false;
}

//-----------------------------------------------------------------------------
int __open( const char* path, int oflags, const mode_t& mode ) {
  return open( path, oflags, mode );

/*if( open( path, oflags, mode ) == -1 ) {
    errno{EACCES,EDQUOT,EEXIST,EFAULT,EFBIG,EINTR,EISDIR,ELOOP,EMFILE,ENAMETOOLONG,ENFILE,ENODEV,ENOENT,ENOMEM,ENOSPC,ENOTDIR,ENXIO,EOVERFLOW,EPERM,EROFS,ETXTBSY,EWOULDBLOCK}
  }*/
}

//-----------------------------------------------------------------------------
void __close( const int& fd ) {
  close( fd );
}

//-----------------------------------------------------------------------------
ssize_t __read( int fd, void* buffer, size_t bytes ) {
  return read( fd, buffer, bytes );

/*if( read( fd, buffer, bytes ) == -1 ) {
    errno{EAGAIN, EBADF, EFAULT, EINVAL, EINTR, EIO, EISDIR}
  }*/
}

//-----------------------------------------------------------------------------
ssize_t __write( int fd, void* buffer, size_t bytes ) {
  return write( fd, buffer, bytes );

/*if( write( fd, buffer, bytes ) == -1 ) {
   errno{EAGAIN, EBADF, EDESTADDRREQ, EDQUOT, EFAULT, EFBIG, EINVAL, EINTR, EIO, ENOSPC, EPIPE
  }*/
}

//-----------------------------------------------------------------------------

