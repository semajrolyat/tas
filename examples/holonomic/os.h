#ifndef _OS_H_
#define _OS_H_

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>             //POSIX
#include <fcntl.h>              //POSIX
#include <sys/time.h>           //POSIX
#include <sys/resource.h>       //POSIX
#include <sched.h>              //POSIX
#include <dlfcn.h>              //POSIX
#include <sys/types.h>          //POSIX
#include <sys/mman.h>           //POSIX
#include <sys/stat.h>           //POSIX

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>

//-----------------------------------------------------------------------------

#ifndef SYS_CLOSE_FD
#define SYS_CLOSE_FD
/// Aliasing the system close(2) function.  Necessary because the buffer has
/// a close method and the compiler doesn't want to use the system function
/// because they have the same name but different signatures
void sys_close_fd( const int& fd );
#endif

//-----------------------------------------------------------------------------
ssize_t sys_write_fd( int fd, const void* buf, size_t count );

//-----------------------------------------------------------------------------
ssize_t sys_read_fd( int fd, void* buf, size_t count );


#endif // _OS_H_
