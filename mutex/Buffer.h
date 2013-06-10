#ifndef BUFFER_H
#define BUFFER_H

#include <sys/mman.h>
#include <fcntl.h>
#include <assert.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include <stdio.h>
#include <cstring>


struct BufferMessage {
    int value;
};

const char* SHM_BUFFER_NAME = "/proc_buffer";

class Buffer {
public:
    Buffer( pthread_mutex_t *mutex ) {
        assert( mutex != NULL );

        this->mutex = mutex;
    }

    virtual ~Buffer( void ) { }

    BufferMessage*      buffer;
    void*               shm_addr;
    int                 fd_buffer;
    pthread_mutex_t     *mutex;

    void create( void ) {
        fd_buffer = shm_open( SHM_BUFFER_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR );
        printf( "created fd for buffer\n" );

        ftruncate( fd_buffer, sizeof(BufferMessage) );
        if( fd_buffer != -1 ) {

            shm_addr = mmap( NULL, sizeof(BufferMessage), PROT_READ | PROT_WRITE, MAP_SHARED, fd_buffer, 0 );
            if( shm_addr != MAP_FAILED ) {
                printf( "mmapped buffer\n" );

                buffer = static_cast<BufferMessage*> (shm_addr);

                //msync( shm_addr, sizeof(BufferMessage), MS_SYNC | MS_INVALIDATE );
            } else {
                perror("mmap()");
            }
        } else {
            perror("shm_open()");
        }
    }

    void open( void ) {
        fd_buffer = shm_open( SHM_BUFFER_NAME, O_RDWR, S_IRUSR | S_IWUSR );
        ftruncate( fd_buffer, sizeof(BufferMessage) );
        if( fd_buffer != -1 ) {

            shm_addr = mmap( NULL, sizeof(BufferMessage), PROT_READ | PROT_WRITE, MAP_SHARED, fd_buffer, 0 );
            if( shm_addr != MAP_FAILED ) {
                buffer = static_cast<BufferMessage*> (shm_addr);

                //msync( shm_addr, sizeof(BufferMessage), MS_SYNC | MS_INVALIDATE );
            } else {
                perror("mmap()");
            }
        } else {
            perror("shm_open()");
        }
    }


    void close( void ) {
        if( munmap( shm_addr, sizeof(pthread_mutex_t) ) )
            perror("munmap()");

        shm_unlink( SHM_BUFFER_NAME );
    }

    void write( const BufferMessage& msg ) {
        pthread_mutex_lock( mutex );

        //memcpy( &buffer, &msg, sizeof(BufferMessage) );
        buffer->value = msg.value;

        msync( shm_addr, sizeof(BufferMessage), MS_SYNC );

        pthread_mutex_unlock( mutex );
    }

    BufferMessage read( void ) {
        pthread_mutex_lock( mutex );

        BufferMessage msg;

        //memcpy( &msg, &buffer, sizeof(BufferMessage) );
        msg.value = buffer->value;

        pthread_mutex_unlock( mutex );

        return msg;
    }
};

#endif // BUFFER_H
