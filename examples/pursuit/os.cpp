#include "os.h"

#include <vector>
#include <assert.h>

#include <sys/select.h>
#include <sys/types.h>
//-----------------------------------------------------------------------------
void __select( const std::vector<int>& fds, fd_set& channels ) {
  int max_fd = 0;

  assert( fds.size() > 0 );

  FD_ZERO( &channels );
  for( int i = 0; i < fds.size(); i++ ) {
    max_fd = std::max( max_fd, fds[i] );
    FD_SET( fds[i], &channels );
  }
  select( max_fd + 1, &channels, NULL, NULL, NULL );
}

//-----------------------------------------------------------------------------
bool __isset( const int& fd, fd_set& channels ) {
  if( FD_ISSET( fd, &channels ) ) return true;
  return false;
}

//-----------------------------------------------------------------------------
void __close( const int& fd ) {
  close( fd );
}

//-----------------------------------------------------------------------------
ssize_t __read( int fd, void* buffer, size_t bytes ) {
  return read( fd, buffer, bytes );
}

//-----------------------------------------------------------------------------
ssize_t __write( int fd, void* buffer, size_t bytes ) {
  return write( fd, buffer, bytes );
}

//-----------------------------------------------------------------------------

