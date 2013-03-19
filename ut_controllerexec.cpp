/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

Very simple unit test/prototype to validate controller process creation when
running the external controller program
-----------------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <signal.h>
#include <stdexcept>
#include <sys/time.h>
#include <sys/resource.h>
#include <errno.h>

#include <fstream>

//-----------------------------------------------------------------------------

#define _DEBUG_TO_STDOUT 1

//-----------------------------------------------------------------------------

static int sim_pid;
static int sim_priority;

static int controller_pid;

std::ofstream controller_log;

//-----------------------------------------------------------------------------

void fork_controller() {

    controller_pid = fork();
    if (controller_pid < 0) {
        throw std::runtime_error( "Failed to fork controller." ) ;
    }
    if (controller_pid == 0) {
        controller_pid = getpid();

        int priority_result = setpriority( PRIO_PROCESS, controller_pid, sim_priority + 1 );
        int controller_priority = getpriority( PRIO_PROCESS, controller_pid );

        controller_log.exceptions( std::ofstream::failbit );

        try {
            controller_log.open("controller.log");
        } catch (std::ofstream::failure ex) {
            printf( "FAILED TO OPEN STREAM.\n" );  // Likely won't print to console
        }

        if( priority_result < 0 ) {
            controller_log << "ERROR: Failed to set priority: " << priority_result << "\n";
        } else {
            controller_log << "Controller PID: " << controller_pid << "\n";
            controller_log << "Controller Priority: " << controller_priority << "\n";
        }

        // execute the controller program
        execl( "controller", "contoller", 0 );

        // exit on fail to exec
        _exit(1);
    }
}

//-----------------------------------------------------------------------------
// Simulator Entry Point
//-----------------------------------------------------------------------------
/**
  Simulator Entry Point
*/
int main( int argc, char* argv[] ) {

    controller_pid = 0;

    sim_pid = getpid();
    sim_priority = getpriority( PRIO_PROCESS, sim_pid );

    fork_controller();

    while(1) {

    }

    return 0;
}
