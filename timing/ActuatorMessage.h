#ifndef _ACTUATORMESSAGE_H_
#define _ACTUATORMESSAGE_H_

//-----------------------------------------------------------------------------

#include <string.h>
#include <iostream>

//-----------------------------------------------------------------------------

typedef double Real;

//-----------------------------------------------------------------------------
// Structures to support ActuatorMessage
//-----------------------------------------------------------------------------
enum ActuatorMessageTypes {
    MSG_TYPE_UNDEFINED,
    MSG_TYPE_COMMAND,
    MSG_TYPE_STATE_REQUEST,
    MSG_TYPE_STATE_REPLY
};

/**
  Header data the composes an ActuatorMessage
*/
struct ActuatorMessageHeader {
    int publisher;
    ActuatorMessageTypes type;
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
        header.publisher = 0;
        header.type = MSG_TYPE_UNDEFINED;

        state.position = 0.0;
        state.velocity = 0.0;
        state.time = 0.0;

        command.torque = 0.0;
    }

    //-------------------------------------------------------------------------
    /// Copy Constructor
    ActuatorMessage( const ActuatorMessage& msg ) {
        header.publisher = msg.header.publisher;
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
        std::cout << "publisher=" << header.publisher;
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
#include <unistd.h>
#include <cstring>

//-----------------------------------------------------------------------------

enum ActuatorMessageBufferErrors {
    ERROR_NONE = 0,
    ERROR_INVALID_KEY,
    ERROR_INVALID_OWNER,
    ERROR_COULD_NOT_OPEN_SHARED_MEMORY,
    ERROR_COULD_NOT_ATTACH_TO_SHARED_MEMORY
};

//-----------------------------------------------------------------------------

/*
Questions:
How to attach a target identifier to the message so that one message structure
 can be differentiated from another in terms of who is intended to receive the
 message

How to make a variable sized or set of buffers as a single entity such that
 multiple messages can be sent simultaneously through the same buffer

*/

class ActuatorMessageBuffer {
private:
    // Member Data
    key_t               key;                // the key identifier for the shared segment
    ActuatorMessage     *shared_buffer;     // pointer to the shared data segment
    int                 buffer_id;          // the index of the buffer returned by shmget
    bool                is_double_buffered; // whether or not this buffer is double buffered
public:

    // Member Data
    int                 owner;              // the process identifier where this class is instantiated

    //-------------------------------------------------------------------------
    // Constructor
    //-------------------------------------------------------------------------
    /// Default Constructor
    ActuatorMessageBuffer( ) {
        this->key = 0;
        this->owner = 0;
    }

    //-------------------------------------------------------------------------
    /// Real Constructor
    ActuatorMessageBuffer( const key_t& key, const int& owner ) {
        this->key = key;
        this->owner = owner;
    }

    //-------------------------------------------------------------------------
    // Destructor
    //-------------------------------------------------------------------------
    virtual ~ActuatorMessageBuffer( void ) { }

    //-------------------------------------------------------------------------
    // 'Device' Operations
    //-------------------------------------------------------------------------
    // Open the shared memory 'device'
    int open( void ) {
        if( key == 0 ) return ERROR_INVALID_KEY;
        if( owner == 0 ) return ERROR_INVALID_OWNER;


        // get the shared memory segment\
        // Note: open for writing (0666) and create
        if( ( buffer_id = shmget( (key_t)key, sizeof(ActuatorMessage), IPC_CREAT | 0666 ) ) < 0 ) {
            perror("shmget");
            return ERROR_COULD_NOT_OPEN_SHARED_MEMORY;
        }

        // attach to the shared memory segment
        if( ( shared_buffer = (ActuatorMessage*)shmat( buffer_id, NULL, 0 ) ) == (ActuatorMessage*) -1 ) {
            perror("shmat");
            return ERROR_COULD_NOT_ATTACH_TO_SHARED_MEMORY;
        }
        return ERROR_NONE;
    }

    //-------------------------------------------------------------------------
    // Close the shared memory 'device'
    void close( void ) {
        // detach from the shared memory segment
        shmdt( shared_buffer );

        // mark for deletion
        shmctl( buffer_id, IPC_RMID, NULL );
    }

    //-------------------------------------------------------------------------
    // Write to the shared memory 'device'
    void write( const ActuatorMessage& msg ) {
        // copy the state and commands first
        memcpy( &shared_buffer->state, &msg.state, sizeof(struct ActuatorMessageState) );
        memcpy( &shared_buffer->command, &msg.command, sizeof(struct ActuatorMessageCommand) );
        // write the header (in particular publisher) LAST because reading is listening for a
        // change to publisher and don't want to trigger the reading of incomplete data
        // So copy each property in header individually
        shared_buffer->header.type = msg.header.type;
        shared_buffer->header.publisher = owner;

        // Should ultimately be protected with a mutex
    }

    //-------------------------------------------------------------------------
    // Read from the shared memory 'device'
    ActuatorMessage read( void ) {
        // loop until another publisher writes into the buffer
        // publisher is initialized to 0 or when a command is written a publisher gets tagged
        while( shared_buffer->header.publisher == owner || shared_buffer->header.publisher == NULL ) {
            // block until someone writes
            usleep(1);
        }

        // define a command to return
        ActuatorMessage msg;
        // unlike in write() don't need to be too particular about order YET.
        // So copy all the header and the state info out with memcpy
        memcpy( &msg.header, &shared_buffer->header, sizeof(struct ActuatorMessageHeader) );
        memcpy( &msg.state, &shared_buffer->state, sizeof(struct ActuatorMessageState) );
        memcpy( &msg.command, &shared_buffer->command, sizeof(struct ActuatorMessageCommand) );

        return msg;
    }

    // NOTE: Remember the suggestion on creating a double buffer.

};

//-----------------------------------------------------------------------------

#endif // _ACTUATORMESSAGE_H_
