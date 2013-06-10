
#include <unistd.h>
#include <sys/types.h>
#include <stdexcept>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/ipc.h>
#include <sys/resource.h>
//#include <dlfcn.h>
#include <stdio.h>
#include <sys/mman.h>

#include "Buffer.h"

//-----------------------------------------------------------------------------

pid_t proc1_pid;
pid_t proc2_pid;

void* shm_addr;
pthread_mutex_t* mutex;
int fd_mutex;

const char* SHM_MUTEX_NAME = "/proc_mutex";

int fd_proc1_to_proc2[2];
int fd_proc2_to_proc1[2];

//-----------------------------------------------------------------------------

void create_mutex( void ) {
    pthread_mutexattr_t mutexattr;

    //fd_mutex = shm_open( SHM_MUTEX_NAME, O_CREAT | O_RDWR, 0640 );
    //fd_mutex = shm_open( SHM_MUTEX_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR | S_IXUSR );
    //fd_mutex = shm_open( SHM_MUTEX_NAME, O_CREAT | O_RDWR | O_EXCL, S_IRUSR | S_IWUSR );
    fd_mutex = shm_open( SHM_MUTEX_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR );
    ftruncate( fd_mutex, sizeof(pthread_mutex_t) );
    if( fd_mutex != -1 ) {
        printf( "(proc1) opened shared region\n" );

        //shm_addr = mmap( NULL, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE | PROT_EXEC, MAP_SHARED, fd_mutex, 0 );
        shm_addr = mmap( NULL, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd_mutex, 0 );
        if( shm_addr != MAP_FAILED ) {
            printf( "(proc1) mmapped shared region\n" );

            mutex = static_cast<pthread_mutex_t*> (shm_addr);
            printf( "(proc1) mutex assigned: 0x%x\n", mutex );

            pthread_mutexattr_init( &mutexattr );
            printf( "(proc1) mutex attributes initialized\n" );

            pthread_mutexattr_setpshared( &mutexattr, PTHREAD_PROCESS_SHARED );
            printf( "(proc1) mutex attributes set\n" );

            int result = pthread_mutex_init( mutex, &mutexattr );
            if( result != 0 ) {
                printf( "(proc1) ERROR : Failed to initialize mutex!\n" );
            }
            printf( "(proc1) mutex initialized\n" );

            pthread_mutexattr_destroy( &mutexattr );
            printf( "(proc1) mutex attributes cleaned up\n" );

            msync( shm_addr, sizeof(pthread_mutex_t), MS_SYNC | MS_INVALIDATE );

            printf( "(proc1) success\n" );
        } else {
            perror("mmap()");
        }
    } else {
        perror("shm_open()");
    }
}

//-----------------------------------------------------------------------------

void create_buffer( void ) {

}

//-----------------------------------------------------------------------------

void fork_proc2( void ) {
    proc1_pid = getpid( );

    proc2_pid = 0;
    proc2_pid = fork( );
    if( proc2_pid < 0 ) {
        throw std::runtime_error( "Failed to fork controller." ) ;
    }
    if( proc2_pid == 0 ) {

        proc2_pid = getpid( );

        close( 100 );
        dup2( fd_proc1_to_proc2[0], 100 );
        close( 101 );
        dup2( fd_proc1_to_proc2[1], 101 );

        close( 102 );
        dup2( fd_proc2_to_proc1[0], 102 );
        close( 103 );
        dup2( fd_proc2_to_proc1[1], 103 );

        execl( "mutex_proc2", "proc2", 0 );
    }
}

//-----------------------------------------------------------------------------

void initialize_pipes( void ) {
    int flags;

    if( pipe( fd_proc1_to_proc2 ) != 0 )
        throw std::runtime_error( "Failed to open fd_proc1_to_proc2 pipe." ) ;
    flags = fcntl( fd_proc1_to_proc2[0], F_GETFL, 0 );
    fcntl( fd_proc1_to_proc2[0], F_SETFL, flags );
    flags = fcntl( fd_proc1_to_proc2[1], F_GETFL, 0 );
    fcntl( fd_proc1_to_proc2[1], F_SETFL, flags | O_NONBLOCK );

    if( pipe( fd_proc2_to_proc1 ) != 0 )
        throw std::runtime_error( "Failed to open fd_proc2_to_proc1 pipe." ) ;
    flags = fcntl( fd_proc2_to_proc1[0], F_GETFL, 0 );
    fcntl( fd_proc2_to_proc1[0], F_SETFL, flags );
    flags = fcntl( fd_proc2_to_proc1[1], F_GETFL, 0 );
    fcntl( fd_proc2_to_proc1[1], F_SETFL, flags | O_NONBLOCK );

}

//-----------------------------------------------------------------------------

int main( int argc, char* argv[] ) {

    initialize_pipes( );
    create_mutex( );

    Buffer buffer = Buffer( mutex );

    buffer.create( );

    create_buffer( );

    BufferMessage msg;
    msg.value = 1;

    fork_proc2( );

    char buf = 0;
    buffer.write( msg );
    write( fd_proc1_to_proc2[1], &buf, 1 );

    read( fd_proc2_to_proc1[0], &buf, 1 );
    msg = buffer.read( );
    printf( "(proc1) read message: %d\n", msg.value );

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
