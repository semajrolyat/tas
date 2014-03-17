#ifndef _OS_H_
#define _OS_H_

#include <unistd.h>
#include <sys/select.h>
#include <vector>

//-----------------------------------------------------------------------------

void __select( const std::vector<int>& fds, fd_set& channels );
bool __isset( const int& fd, fd_set& channels );
void __close( const int& fd );
ssize_t __read( int fd, void* buffer, size_t bytes );
ssize_t __write( int fd, void* buffer, size_t bytes );

//-----------------------------------------------------------------------------

#endif
