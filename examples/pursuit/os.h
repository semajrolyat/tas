#ifndef _OS_H_
#define _OS_H_

#include <sys/types.h>
#include <vector>

//-----------------------------------------------------------------------------

void __select( const std::vector<int>& fds, fd_set& channels );
bool __isset( const int& fd, fd_set& channels );
int __open( const char* path, int oflags, const mode_t& mode );
void __close( const int& fd );
ssize_t __read( int fd, void* buffer, size_t bytes );
ssize_t __write( int fd, void* buffer, size_t bytes );

//-----------------------------------------------------------------------------

#endif
