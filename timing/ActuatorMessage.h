/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

ActuatorMessage.h
-----------------------------------------------------------------------------*/

#ifndef _ACTUATORMESSAGE_H_
#define _ACTUATORMESSAGE_H_

//-----------------------------------------------------------------------------

#include <string.h>
#include <iostream>
#include <pthread.h>
#include <assert.h>

//-----------------------------------------------------------------------------

typedef double Real;

//-----------------------------------------------------------------------------
// Structures to support ActuatorMessage
//-----------------------------------------------------------------------------
enum ActuatorMessageTypes {
    MSG_TYPE_UNDEFINED,
    MSG_TYPE_COMMAND,
    MSG_TYPE_NO_COMMAND,
    MSG_TYPE_STATE_REQUEST,
    MSG_TYPE_STATE_REPLY
};

/**
  Header data the composes an ActuatorMessage
*/
struct ActuatorMessageHeader {
    int pid;					// ProcessIDentifier
    int tid;					// ThreadIDentifier
    ActuatorMessageTypes type;	// messageType
    timespec ts;				// TimeStamp
};

//-----------------------------------------------------------------------------
/**
  State data the composes an ActuatorMessage
*/
struct ActuatorMessageState {
    Real time;
    Real position;
    Real velocity;
};

//-----------------------------------------------------------------------------
/**
  Command data the composes an ActuatorMessage
*/
struct ActuatorMessageCommand {
    Real torque;
};

//-----------------------------------------------------------------------------
// ActuatorMessage
//-----------------------------------------------------------------------------
/**
  Message passed between controllers and dynamics to communicate actuation
  information
*/
class ActuatorMessage {
public:
    //-------------------------------------------------------------------------
    // Member Data
    //-------------------------------------------------------------------------
    ActuatorMessageHeader header;
    ActuatorMessageState state;
    ActuatorMessageCommand command;

    //-------------------------------------------------------------------------
    // Constructors
    //-------------------------------------------------------------------------
    /// Default Constructor
    ActuatorMessage( void ) {
        header.pid = 0;
        header.tid = 0;
        header.ts.tv_sec = 0;
        header.ts.tv_nsec = 0;
        header.type = MSG_TYPE_UNDEFINED;

        state.position = 0.0;
        state.velocity = 0.0;
        state.time = 0.0;

        command.torque = 0.0;
    }

    //-------------------------------------------------------------------------
    /// Copy Constructor
    ActuatorMessage( const ActuatorMessage& msg ) {
        header.pid = msg.header.pid;
        header.tid = msg.header.tid;
        header.ts.tv_sec = msg.header.ts.tv_sec;
        header.ts.tv_nsec = msg.header.ts.tv_nsec;
        header.type = msg.header.type;

        state.position = msg.state.position;
        state.velocity = msg.state.velocity;
        state.time = msg.state.time;

        command.torque = msg.command.torque;
    }
/*
    //-------------------------------------------------------------------------
    /// Publisher Override (to Republish) Copy Constructor
    ActuatorMessage( const int& publisher, const ActuatorMessage& msg ) {
        header.publisher = publisher;
        header.type = msg.header.type;

        state.position = msg.state.position;
        state.velocity = msg.state.velocity;
        state.time = msg.state.time;

        command.torque = msg.command.torque;
    }
*/
    //-------------------------------------------------------------------------
    // Destructor
    //-------------------------------------------------------------------------
    /// Destructor
    virtual ~ActuatorMessage( void ) { }

    //-------------------------------------------------------------------------
    // Utilities
    //-------------------------------------------------------------------------
    /// Print the datastructure to standard out
    void print( void ) {
        std::cout << "pid=" << header.pid;
        std::cout << ", tid=" << header.tid;
        std::cout << ", ts.tv_sec=" << header.ts.tv_sec;
        std::cout << ", ts.tv_nsec=" << header.ts.tv_nsec;
        std::cout << ", type=" << header.type;
       
        std::cout << ", time=" << state.time;
        std::cout << ", position=" << state.position;
        std::cout << ", velocity=" << state.velocity;

        std::cout << ", torque=" << command.torque;

        std::cout << std::endl;
    }
};

//-----------------------------------------------------------------------------
// Shared Buffer for transmitting ActuatorMessages between processes
//-----------------------------------------------------------------------------

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>

//-----------------------------------------------------------------------------

enum ActuatorMessageBufferErrors {
    ERROR_NONE = 0,
    ERROR_MAPPING_MEMORY,
    ERROR_UNMAPPING_MEMORY,
    ERROR_OPENING_SHARED_MEMORY,
    ERROR_UNLINKING_SHARED_MEMORY,
    ERROR_SYNCING_MEMORY,
    ERROR_INITIALIZING_MUTEX
};

//-----------------------------------------------------------------------------

#define ACTUATOR_MESSAGE_BUFFERS_MAX 2
const char* ACTUATOR_MESSAGE_BUFFER_NAME = "/amsgbuffer";
const char* ACTUATOR_MESSAGE_BUFFER_MUTEX_NAME = "/amsgbuffer_mutex";

//-----------------------------------------------------------------------------

class ActuatorMessageBuffer {
private:

    bool                initialized;
    bool                opened;

    std::string         buffer_name;
    std::string         mutex_name;

    int                 fd_mutex;
    pthread_mutex_t     *mutex;

    int                 fd_buffer;
    ActuatorMessage*    buffer;

public:

    //-------------------------------------------------------------------------
    // Constructors
    //-------------------------------------------------------------------------
    /// Default Constructor
    ActuatorMessageBuffer( void ) {
        initialized = false;
        opened = false;
        mutex = NULL;
        buffer = NULL;
    }

    //-------------------------------------------------------------------------

    ActuatorMessageBuffer( const char* buffer_name, const char* mutex_name, const bool& create = false ) {

        assert( buffer_name != NULL && mutex_name != NULL );

        this->buffer_name = buffer_name;
        this->mutex_name = mutex_name;

        if( init_mutex( create ) != ERROR_NONE ) {
            // major problem.  No option but to bomb out.
            printf( "failed to initialize mutex\n" );
        }
        if( init_buffer( create ) != ERROR_NONE ) {
            // major problem.  No option but to bomb out.
            printf( "failed to initialize buffer\n" );
        }

        initialized = true;
    }

    //-------------------------------------------------------------------------
    // Destructor
    //-------------------------------------------------------------------------

    virtual ~ActuatorMessageBuffer( void ) { }

private:
    //-------------------------------------------------------------------------

    int init_mutex( const bool& create = false ) {

        void* shm_mutex_addr;
        pthread_mutexattr_t mutexattr;

        if( create )
            fd_mutex = shm_open( mutex_name.c_str( ), O_CREAT | O_RDWR, S_IRUSR | S_IWUSR );
        else
            fd_mutex = shm_open( mutex_name.c_str( ), O_RDWR, S_IRUSR | S_IWUSR );

        ftruncate( fd_mutex, sizeof(pthread_mutex_t) );

        if( fd_mutex != -1 ) {
            shm_mutex_addr = mmap( NULL, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd_mutex, 0 );
            if( shm_mutex_addr != MAP_FAILED ) {
                mutex = static_cast<pthread_mutex_t*> ( shm_mutex_addr );
                if( create ) {
                    pthread_mutexattr_init( &mutexattr );

                    pthread_mutexattr_setpshared( &mutexattr, PTHREAD_PROCESS_SHARED );

                    if( pthread_mutex_init( mutex, &mutexattr ) != 0 ) {
                        return ERROR_INITIALIZING_MUTEX;
                    }
                    pthread_mutexattr_destroy( &mutexattr );

                    if( msync( shm_mutex_addr, sizeof(pthread_mutex_t), MS_SYNC | MS_INVALIDATE ) != 0 ) {
                        perror( "msync()" );
                        return ERROR_SYNCING_MEMORY;
                    }
                }
            } else {
                perror( "mmap()" );
                return ERROR_MAPPING_MEMORY;
            }
        } else {
            perror( "shm_open()" );
            return ERROR_OPENING_SHARED_MEMORY;
        }
        return ERROR_NONE;

        /*
        if( create ) {
            // Creating a mutex in shared memory

            fd_mutex = shm_open( mutex_name, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR );
            ftruncate( fd_mutex, sizeof(pthread_mutex_t) );
            if( fd_mutex != -1 ) {
                //printf( "opened shared region\n" );

                shm_mutex_addr = mmap( NULL, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd_mutex, 0 );
                if( shm_mutex_addr != MAP_FAILED ) {
                    //printf( "mmapped shared region\n" );

                    mutex = static_cast<pthread_mutex_t*> ( shm_mutex_addr );
                    //printf( "mutex assigned: 0x%x\n", mutex );

                    pthread_mutexattr_init( &mutexattr );
                    //printf( "mutex attributes initialized\n" );

                    pthread_mutexattr_setpshared( &mutexattr, PTHREAD_PROCESS_SHARED );
                    //printf( "mutex attributes set\n" );

                    if( pthread_mutex_init( mutex, &mutexattr ) != 0 ) {
                        return ERROR_INITIALIZING_MUTEX;
                    }
                    //printf( "mutex initialized\n" );

                    pthread_mutexattr_destroy( &mutexattr );
                    //printf( "mutex attributes cleaned up\n" );

                    if( msync( shm_mutex_addr, sizeof(pthread_mutex_t), MS_SYNC | MS_INVALIDATE ) != 0 ) {
                        perror( "msync()" );
                        return ERROR_SYNCING_MEMORY;
                    }

                    //printf( "successfully created mutex\n" );
                } else {
                    perror( "mmap()" );
                    return ERROR_MAPPING_MEMORY;
                }
            } else {
                perror( "shm_open()" );
                return ERROR_OPENING_SHARED_MEMORY;
            }
        } else {
            // Attaching to an existing mutex.  Some process must create it before
            // this region is invoked.  Basically, the parent process should
            // invoke this code before forking or spawning threads with create
            // and any subsequent child threads or processes invoke this code without
            // create

            fd_mutex = shm_open( mutex_name, O_RDWR, S_IRUSR | S_IWUSR );
            ftruncate( fd_mutex, sizeof(pthread_mutex_t) );
            if( fd_mutex != -1 ) {
                //printf( "opened mutex shared region\n" );

                shm_mutex_addr = mmap( NULL, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd_mutex, 0 );
                if( shm_mutex_addr != MAP_FAILED ) {
                    //printf( "mmapped mutex shared region\n" );

                    mutex = static_cast<pthread_mutex_t*> ( shm_mutex_addr );
                    //printf( "mutex assigned: 0x%x\n", mutex );

                    //printf( "successfully attached to mutex\n" );
                } else {
                    perror( "mmap()" );
                    return ERROR_MAPPING_MEMORY;
                }
            } else {
                perror( "shm_open()" );
                return ERROR_OPENING_SHARED_MEMORY;
            }
        }
        return ERROR_NONE;
        */

    }

    //-------------------------------------------------------------------------

    int delete_mutex( void ) {
        if( munmap( (void*)mutex, sizeof(pthread_mutex_t) ) ) {
            perror( "munmap()" );
            return ERROR_UNMAPPING_MEMORY;
        }

        if( shm_unlink( mutex_name.c_str( ) ) != 0 ) {
            return ERROR_UNLINKING_SHARED_MEMORY;
        }

        return ERROR_NONE;
    }

    //-------------------------------------------------------------------------

    int init_buffer( const bool& create = false ) {

        void* shm_buffer_addr;

        if( create )
            fd_buffer = shm_open( buffer_name.c_str( ), O_CREAT | O_RDWR, S_IRUSR | S_IWUSR );
        else
            fd_buffer = shm_open( buffer_name.c_str( ), O_RDWR, S_IRUSR | S_IWUSR );

        ftruncate( fd_buffer, sizeof(ActuatorMessage) );

        if( fd_buffer != -1 ) {
            shm_buffer_addr = mmap( NULL, sizeof(ActuatorMessage), PROT_READ | PROT_WRITE, MAP_SHARED, fd_buffer, 0 );
            if( shm_buffer_addr != MAP_FAILED ) {
                buffer = static_cast<ActuatorMessage*> ( shm_buffer_addr );
            } else {
                perror( "mmap()" );
                return ERROR_MAPPING_MEMORY;
            }
        } else {
            perror( "shm_open()" );
            return ERROR_OPENING_SHARED_MEMORY;
        }
        return ERROR_NONE;

        /*
        if( create ) {
            fd_buffer = shm_open( buffer_name, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR );
            ftruncate( fd_buffer, sizeof(ActuatorMessage) );
            if( fd_buffer != -1 ) {

                shm_buffer_addr = mmap( NULL, sizeof(ActuatorMessage), PROT_READ | PROT_WRITE, MAP_SHARED, fd_buffer, 0 );
                if( shm_buffer_addr != MAP_FAILED ) {
                    buffer = static_cast<ActuatorMessage*> ( shm_buffer_addr );
                } else {
                    perror( "mmap()" );
                    return ERROR_MAPPING_MEMORY;
                }
            } else {
                perror( "shm_open()" );
                return ERROR_OPENING_SHARED_MEMORY;
            }
        } else {
            assert( initialized == true );

            fd_buffer = shm_open( buffer_name, O_RDWR, S_IRUSR | S_IWUSR );
            ftruncate( fd_buffer, sizeof(ActuatorMessage) );
            if( fd_buffer != -1 ) {
                shm_buffer_addr = mmap( NULL, sizeof(ActuatorMessage), PROT_READ | PROT_WRITE, MAP_SHARED, fd_buffer, 0 );
                if( shm_buffer_addr != MAP_FAILED ) {
                    buffer = static_cast<ActuatorMessage*> ( shm_buffer_addr );
                } else {
                    perror( "mmap()" );
                    return ERROR_MAPPING_MEMORY;
                }

                opened = true;
            } else {
                perror( "shm_open()" );
                return ERROR_OPENING_SHARED_MEMORY;
            }
        }
        return ERROR_NONE;
        */
    }

    //-------------------------------------------------------------------------

    int delete_buffer( void ) {
        if( munmap( (void*)buffer, sizeof(ActuatorMessage) ) ) {
            perror("munmap()");
            return ERROR_UNMAPPING_MEMORY;
        }

        if( shm_unlink( buffer_name.c_str( ) ) != 0 ) {
            return ERROR_UNLINKING_SHARED_MEMORY;
        }

        return ERROR_NONE;
    }

    //-------------------------------------------------------------------------

public:

    //-------------------------------------------------------------------------

    int open( void ) {
        assert( initialized );

        opened = true;

        return ERROR_NONE;
    }

    //-------------------------------------------------------------------------

    void close( void ) {

        opened = false;
    }

    //-------------------------------------------------------------------------

    int write( const ActuatorMessage& msg ) {
        assert( opened == true );

        pthread_mutex_lock( mutex );

        memcpy( &buffer->state, &msg.state, sizeof(struct ActuatorMessageState) );
        memcpy( &buffer->command, &msg.command, sizeof(struct ActuatorMessageCommand) );
        memcpy( &buffer->header, &msg.header, sizeof(struct ActuatorMessageHeader) );

        if( msync( (void*)buffer, sizeof(ActuatorMessage), MS_SYNC ) != 0 ) {
            perror( "msync()" );
            return ERROR_SYNCING_MEMORY;
        }

        pthread_mutex_unlock( mutex );

        return ERROR_NONE;
    }

    //-------------------------------------------------------------------------

    ActuatorMessage read( void ) {
        assert( opened == true );

        pthread_mutex_lock( mutex );

        ActuatorMessage msg;

        memcpy( &msg.header, &buffer->header, sizeof(struct ActuatorMessageHeader) );
        memcpy( &msg.state, &buffer->state, sizeof(struct ActuatorMessageState) );
        memcpy( &msg.command, &buffer->command, sizeof(struct ActuatorMessageCommand) );

        pthread_mutex_unlock( mutex );

        return msg;
    }

    //-------------------------------------------------------------------------

};

//-----------------------------------------------------------------------------

#endif // _ACTUATORMESSAGE_H_
