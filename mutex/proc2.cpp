#include <unistd.h>
#include <sys/types.h>
#include <stdexcept>
#include <stdio.h>

#include "Buffer.h"

//-----------------------------------------------------------------------------

void* shm_addr;
pthread_mutex_t* mutex;
int fd_mutex;

const char* SHM_MUTEX_NAME = "/proc_mutex";

int fd_proc1_to_proc2[2];
int fd_proc2_to_proc1[2];

//-----------------------------------------------------------------------------

void attach_mutex( void ) {
    //fd_mutex = shm_open( SHM_MUTEX_NAME, O_CREAT | O_RDWR, 0640 );
    //fd_mutex = shm_open( SHM_MUTEX_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR | S_IXUSR );
    //fd_mutex = shm_open( SHM_MUTEX_NAME, O_CREAT | O_RDWR | O_EXCL, S_IRUSR | S_IWUSR );
    //fd_mutex = shm_open( SHM_MUTEX_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR );
    fd_mutex = shm_open( SHM_MUTEX_NAME, O_RDWR, S_IRUSR | S_IWUSR );
    ftruncate( fd_mutex, sizeof(pthread_mutex_t) );
    if( fd_mutex != -1 ) {
        printf( "(proc2) opened shared region\n" );

        //shm_addr = mmap( NULL, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE | PROT_EXEC, MAP_SHARED, fd_mutex, 0 );
        shm_addr = mmap( NULL, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd_mutex, 0 );
        if( shm_addr != MAP_FAILED ) {
            printf( "(proc2) mmapped shared region\n" );

            mutex = static_cast<pthread_mutex_t*> (shm_addr);
            printf( "(proc2) mutex assigned: 0x%x\n", mutex );

            printf( "(proc2) success\n" );
        } else {
            perror("mmap()");
        }
    } else {
        perror("shm_open()");
    }
}

//-----------------------------------------------------------------------------

int main( int argc, char* argv[] ) {

    fd_proc1_to_proc2[0] = 100;
    fd_proc1_to_proc2[1] = 101;

    fd_proc2_to_proc1[0] = 102;
    fd_proc2_to_proc1[1] = 103;

    attach_mutex( );

    Buffer buffer = Buffer( mutex );

    buffer.open( );

    char buf;
    read( fd_proc1_to_proc2[0], &buf, 1 );
    BufferMessage msg = buffer.read( );
    printf( "(proc2) read message: %d\n", msg.value );

    buf = 0;
    msg.value = 2;
    buffer.write( msg );
    write( fd_proc2_to_proc1[1], &buf, 1 );

    double x = 1.1;
    for( int i = 0; i < 10000; i++ ) {
        x += x;
        usleep(1);
    }

    /*
    if( munmap( shm_addr, sizeof(pthread_mutex_t) ) )
        perror("munmap()");

    shm_unlink( SHM_MUTEX_NAME );
    */
}
