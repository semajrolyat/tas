#include <stdlib.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>

#include "CommandBuffer.h"

int main( void ) {

    int pid = getpid( );

    CommandBuffer cmdbuffer = CommandBuffer( pid );

    Command cmd_received;

    int result = cmdbuffer.open( );
    if( result != 0 ) {
        printf( "Failed to open shared data segment\n" );
        exit(1);
    }

    cmd_received = cmdbuffer.read( );

    printf( "(client) owner : %d : read ", cmdbuffer.owner );
    cmd_received.print( );

    Command cmd_send = Command( cmdbuffer.owner, cmd_received );
    cmd_send.state.torque = 4096.0;

    cmdbuffer.write( cmd_send );

    printf( "(client) owner : %d : wrote ", cmdbuffer.owner );
    cmd_send.print( );

    cmdbuffer.close( );

    return 0;

}


