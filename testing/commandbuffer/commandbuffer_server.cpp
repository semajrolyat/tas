#include <stdlib.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <unistd.h>

#include "CommandBuffer.h"

int main( void ) {

    // get the process id of this process
    int pid = getpid( );
    // instantiate a command buffer
    CommandBuffer cmdbuffer = CommandBuffer( pid );

    // open the command buffer
    int result = cmdbuffer.open( );
    if( result != 0 ) {
        printf( "Failed to open shared data segment\n" );
        exit(1);
    }

    // define a command to send through the buffer
    Command cmd_send;
    cmd_send.state.position = 256.0;
    cmd_send.state.velocity = 512.0;
    cmd_send.state.time = 1024.0;

    // write the command to the buffer
    cmdbuffer.write( cmd_send );

    // report the command written to the buffer
    printf( "(server) owner : %d : wrote ", cmdbuffer.owner );
    cmd_send.print( );

    // read a response from the command buffer
    // Note: blocks until a command is read
    Command cmd_received = cmdbuffer.read( );

    // report the response command
    printf( "(server) owner : %d : read ", cmdbuffer.owner );
    cmd_received.print( );

    // close/detach the buffer
    cmdbuffer.close( );

    return 0;
}


