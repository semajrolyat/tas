#ifndef _COMMANDBUFFER_H_
#define _COMMANDBUFFER_H_

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include <cstring>

//-----------------------------------------------------------------------------

#define CTLDATA_KEY 8811

typedef double Real;

//-----------------------------------------------------------------------------

struct CommandHeader {
    int publisher;
};

struct CommandState {
    // input
    Real position;
    Real velocity;
    Real time;
    // output
    Real torque;
};

class Command {
public:
    CommandHeader header;
    CommandState state;

    Command( void ) {
        header.publisher = 0;

        state.position = 0.0;
        state.velocity = 0.0;
        state.time = 0.0;

        state.torque = 0.0;
    }

    Command( const Command& cmd ) {
        header.publisher = cmd.header.publisher;

        state.position = cmd.state.position;
        state.velocity = cmd.state.velocity;
        state.time = cmd.state.time;

        state.torque = cmd.state.torque;
    }

    Command( const int& publisher, const Command& cmd ) {
        header.publisher = publisher;

        state.position = cmd.state.position;
        state.velocity = cmd.state.velocity;
        state.time = cmd.state.time;

        state.torque = cmd.state.torque;
    }

    virtual ~Command( void ) { }

    //-------------------------------------------------------------------------
    // Utilities
    //-------------------------------------------------------------------------
    // Echo the datastructure to the console
    void print( void ) {
        printf( "publisher=%d, time=%lf, position=%lf, velocity=%lf, torque=%lf\n", header.publisher, state.time, state.position, state.velocity, state.torque );
    }

};

//-----------------------------------------------------------------------------

class CommandBuffer {

private:
    // Member Data
    Command     *shared_buffer;     // pointer to the shared data segment
    int         buffer_id;          // the index of the buffer returned by shmget

public:
    // Member Data
    int         owner;              // the process identifier where this class is instantiated

    //-------------------------------------------------------------------------
    // Constructor
    //-------------------------------------------------------------------------
    CommandBuffer( ) {
        this->owner = 0;
    }

    CommandBuffer( const int& owner ) {
        this->owner = owner;
    }

    //-------------------------------------------------------------------------
    // Destructor
    //-------------------------------------------------------------------------
    virtual ~CommandBuffer( void ) { }

    //-------------------------------------------------------------------------
    // 'Device' Operations
    //-------------------------------------------------------------------------
    // Open the shared memory 'device'
    int open( void ) {

        // get the shared memory segment\
        // Note: open for writing (0666) and create
        if( ( buffer_id = shmget( (key_t)CTLDATA_KEY, sizeof(Command), IPC_CREAT | 0666 ) ) < 0 ) {
            perror("shmget");
            return 1;
        }

        // attach to the shared memory segment
        if( ( shared_buffer = (Command*)shmat( buffer_id, NULL, 0 ) ) == (Command*) -1 ) {
            perror("shmat");
            return 1;
        }
        return 0;
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
    void write( const Command& cmd ) {
        // copy the state first
        memcpy( &shared_buffer->state, &cmd.state, sizeof(struct CommandState) );
        // write the publisher LAST because reading is listening for a change to publisher
        // and don't want to trigger the reading of incomplete data
        shared_buffer->header.publisher = owner;

        // Should ultimately be protected with a mutex
    }

    //-------------------------------------------------------------------------
    // Read from the shared memory 'device'
    Command read( void ) {
        // loop until another publisher writes into the buffer
        // publisher is initialized to NULL or when a command is written a publisher gets tagged
        while( shared_buffer->header.publisher == owner || shared_buffer->header.publisher == NULL ) {
            // block until someone writes
            usleep(1);
        }

        // define a command to return
        Command cmd;
        // unlike in write() don't need to be too particular about order YET.
        // So copy all the header and the state info out with memcpy
        memcpy( &cmd.header, &shared_buffer->header, sizeof(struct CommandHeader) );
        memcpy( &cmd.state, &shared_buffer->state, sizeof(struct CommandState) );

        return cmd;
    }

    // NOTE: Remember the suggestion on creating a double buffer.

};

//-----------------------------------------------------------------------------

#endif // _COMMANDBUFFER_H_
