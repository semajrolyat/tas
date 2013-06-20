/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

actuator.h
-----------------------------------------------------------------------------*/

#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

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
enum actuator_msg_e {
    ACTUATOR_MSG_UNDEFINED,
    ACTUATOR_MSG_COMMAND,
    ACTUATOR_MSG_NO_COMMAND,
    ACTUATOR_MSG_REQUEST,
    ACTUATOR_MSG_REPLY
};

//-----------------------------------------------------------------------------

///  Header data
struct actuator_msg_header_t {
    int pid;					// ProcessIDentifier
    int tid;					// ThreadIDentifier
    actuator_msg_e type;			// Message Type
    //timespec ts;				// TimeStamp
    unsigned long long ts;			// TimeStamp
};

//-----------------------------------------------------------------------------

///  State data
struct actuator_msg_state_t {
    Real time;
    Real position;
    Real velocity;
};

//-----------------------------------------------------------------------------

///  Command data
struct actuator_msg_command_t {
    Real torque;
};

//-----------------------------------------------------------------------------
// actuator message
//-----------------------------------------------------------------------------
///  Message passed in system to communicate actuation action and information
class actuator_msg_c {
public:
    //-------------------------------------------------------------------------
    // Member Data
    //-------------------------------------------------------------------------
    actuator_msg_header_t	header;
    actuator_msg_state_t	state;
    actuator_msg_command_t 	command;

    //-------------------------------------------------------------------------
    // Constructors
    //-------------------------------------------------------------------------
    /// Default Constructor
    actuator_msg_c( void ) {
        header.pid = 0;
        header.tid = 0;
        //header.ts.tv_sec = 0;
        //header.ts.tv_nsec = 0;
	header.ts = 0;
        header.type = ACTUATOR_MSG_UNDEFINED;

        state.position = 0.0;
        state.velocity = 0.0;
        state.time = 0.0;

        command.torque = 0.0;
    }

    //-------------------------------------------------------------------------
    /// Copy Constructor
    actuator_msg_c( const actuator_msg_c& msg ) {
        header.pid = msg.header.pid;
        header.tid = msg.header.tid;
        //header.ts.tv_sec = msg.header.ts.tv_sec;
        //header.ts.tv_nsec = msg.header.ts.tv_nsec;
	header.ts = msg.header.ts;
        header.type = msg.header.type;

        state.position = msg.state.position;
        state.velocity = msg.state.velocity;
        state.time = msg.state.time;

        command.torque = msg.command.torque;
    }

    //-------------------------------------------------------------------------
    // Destructor
    //-------------------------------------------------------------------------
    /// Destructor
    virtual ~actuator_msg_c( void ) { }

    //-------------------------------------------------------------------------
    // Utilities
    //-------------------------------------------------------------------------
    /// Print the datastructure to standard out
    void print( void ) {
        std::cout << "pid=" << header.pid;
        std::cout << ", tid=" << header.tid;
        //std::cout << ", ts.tv_sec=" << header.ts.tv_sec;
        //std::cout << ", ts.tv_nsec=" << header.ts.tv_nsec;
        std::cout << ", ts=" << header.ts;
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

/// Aliasing the system close(2) function.  Necessary because the buffer has
/// a close method and the compiler doesn't want to use the system function
/// because they have the same name but different signatures
void sys_close_fd( const int& fd ) {
    close( fd );
}

//-----------------------------------------------------------------------------

enum actuator_msg_buffer_err_e {
    BUFFER_ERROR_NONE = 0,
    BUFFER_ERROR_MAPPING,
    BUFFER_ERROR_UNMAPPING,
    BUFFER_ERROR_OPENING,
    BUFFER_ERROR_UNLINKING,
    BUFFER_ERROR_SYNCING,
    BUFFER_ERROR_MUTEX
};

//-----------------------------------------------------------------------------

#define ACTUATOR_MESSAGE_BUFFERS_MAX 2
const char* ACTUATOR_MSG_BUFFER_NAME = "/amsgbuffer";
const char* ACTUATOR_MSG_BUFFER_MUTEX_NAME = "/amsgbuffer_mutex";

//-----------------------------------------------------------------------------

class actuator_msg_buffer_c {
private:

    bool                initialized;
    bool                opened;

    std::string         buffer_name;
    std::string         mutex_name;

    int                 fd_mutex;
    pthread_mutex_t     *mutex;

    int                 fd_buffer;
    actuator_msg_c*    	buffer;

public:

    //-------------------------------------------------------------------------
    // Constructors
    //-------------------------------------------------------------------------
    /// Default Constructor
    actuator_msg_buffer_c( void ) {
        initialized = false;
        opened = false;
        mutex = NULL;
        buffer = NULL;
    }

    //-------------------------------------------------------------------------

    actuator_msg_buffer_c( const char* buffer_name, const char* mutex_name, const bool& create = false ) {

        assert( buffer_name != NULL && mutex_name != NULL );

        this->buffer_name = buffer_name;
        this->mutex_name = mutex_name;

        if( init_mutex( create ) != BUFFER_ERROR_NONE ) {
            // major problem.  No option but to bomb out.
            printf( "failed to initialize mutex\n" );
        }
        if( init_buffer( create ) != BUFFER_ERROR_NONE ) {
            // major problem.  No option but to bomb out.
            printf( "failed to initialize buffer\n" );
        }

        initialized = true;
    }

    //-------------------------------------------------------------------------
    // Destructor
    //-------------------------------------------------------------------------

    virtual ~actuator_msg_buffer_c( void ) { }

private:
    //-------------------------------------------------------------------------

    actuator_msg_buffer_err_e init_mutex( const bool& create = false ) {

        void* shm_mutex_addr;
        pthread_mutexattr_t mutexattr;

        if( create )
            fd_mutex = shm_open( mutex_name.c_str( ), O_CREAT | O_RDWR, S_IRUSR | S_IWUSR );
        else
            fd_mutex = shm_open( mutex_name.c_str( ), O_RDWR, S_IRUSR | S_IWUSR );

        if( fd_mutex == -1 ) {
            perror( "shm_open()" );
            return BUFFER_ERROR_OPENING;
        }

        ftruncate( fd_mutex, sizeof(pthread_mutex_t) );

        shm_mutex_addr = mmap( NULL, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd_mutex, 0 );

        if( shm_mutex_addr == MAP_FAILED ) {
            perror( "mmap()" );
            sys_close_fd( fd_mutex );
            return BUFFER_ERROR_MAPPING;
        }

        mutex = static_cast<pthread_mutex_t*> ( shm_mutex_addr );
        if( create ) {
            pthread_mutexattr_init( &mutexattr );

            pthread_mutexattr_setpshared( &mutexattr, PTHREAD_PROCESS_SHARED );

            if( pthread_mutex_init( mutex, &mutexattr ) != 0 ) {
                // what other handling should be done in response.  Close the fd?
                return BUFFER_ERROR_MUTEX;
            }
            pthread_mutexattr_destroy( &mutexattr );

            if( msync( shm_mutex_addr, sizeof(pthread_mutex_t), MS_SYNC | MS_INVALIDATE ) != 0 ) {
                // what other handling should be done in response.  Close the fd, release the mutex?
                perror( "msync()" );
                return BUFFER_ERROR_SYNCING;
            }
        }

        return BUFFER_ERROR_NONE;
    }

    //-------------------------------------------------------------------------

    actuator_msg_buffer_err_e delete_mutex( void ) {
        if( munmap( (void*)mutex, sizeof(pthread_mutex_t) ) ) {
            perror( "munmap()" );
            return BUFFER_ERROR_UNMAPPING;
        }

        if( shm_unlink( mutex_name.c_str( ) ) != 0 ) {
            return BUFFER_ERROR_UNLINKING;
        }

        return BUFFER_ERROR_NONE;
    }

    //-------------------------------------------------------------------------

    actuator_msg_buffer_err_e init_buffer( const bool& create = false ) {

        void* shm_buffer_addr;

        if( create )
            fd_buffer = shm_open( buffer_name.c_str( ), O_CREAT | O_RDWR, S_IRUSR | S_IWUSR );
        else
            fd_buffer = shm_open( buffer_name.c_str( ), O_RDWR, S_IRUSR | S_IWUSR );

        if( fd_buffer == -1 ) {
            perror( "shm_open()" );
            return BUFFER_ERROR_OPENING;
        }

        ftruncate( fd_buffer, sizeof(actuator_msg_c) );

        shm_buffer_addr = mmap( NULL, sizeof(actuator_msg_c), PROT_READ | PROT_WRITE, MAP_SHARED, fd_buffer, 0 );

        if( shm_buffer_addr == MAP_FAILED ) {
            perror( "mmap()" );
            sys_close_fd( fd_mutex );
            return BUFFER_ERROR_MAPPING;
        }

        buffer = static_cast<actuator_msg_c*> ( shm_buffer_addr );

        return BUFFER_ERROR_NONE;
    }

    //-------------------------------------------------------------------------

    actuator_msg_buffer_err_e delete_buffer( void ) {
        if( munmap( (void*)buffer, sizeof(actuator_msg_c) ) ) {
            perror("munmap()");
            return BUFFER_ERROR_UNMAPPING;
        }

        if( shm_unlink( buffer_name.c_str( ) ) != 0 ) {
            return BUFFER_ERROR_UNLINKING;
        }

        return BUFFER_ERROR_NONE;
    }

    //-------------------------------------------------------------------------

public:

    //-------------------------------------------------------------------------

    actuator_msg_buffer_err_e open( void ) {
        assert( initialized );

        opened = true;

        return BUFFER_ERROR_NONE;
    }

    //-------------------------------------------------------------------------

    void close( void ) {

        opened = false;
    }

    //-------------------------------------------------------------------------

    actuator_msg_buffer_err_e write( const actuator_msg_c& msg ) {
        assert( opened == true );

        pthread_mutex_lock( mutex );

        memcpy( &buffer->state, &msg.state, sizeof(struct actuator_msg_state_t) );
        memcpy( &buffer->command, &msg.command, sizeof(struct actuator_msg_command_t) );
        memcpy( &buffer->header, &msg.header, sizeof(struct actuator_msg_header_t) );

        if( msync( (void*)buffer, sizeof(actuator_msg_c), MS_SYNC ) != 0 ) {
            perror( "msync()" );
            return BUFFER_ERROR_SYNCING;
        }

        pthread_mutex_unlock( mutex );

        return BUFFER_ERROR_NONE;
    }

    //-------------------------------------------------------------------------

    actuator_msg_c read( void ) {
        assert( opened == true );

        pthread_mutex_lock( mutex );

        actuator_msg_c msg;

        memcpy( &msg.header, &buffer->header, sizeof(struct actuator_msg_header_t) );
        memcpy( &msg.state, &buffer->state, sizeof(struct actuator_msg_state_t) );
        memcpy( &msg.command, &buffer->command, sizeof(struct actuator_msg_command_t) );

        pthread_mutex_unlock( mutex );

        return msg;
    }

    //-------------------------------------------------------------------------

};

//-----------------------------------------------------------------------------

#endif // _ACTUATOR_H_
