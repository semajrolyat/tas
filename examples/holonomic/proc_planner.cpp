/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

planner.cpp
-----------------------------------------------------------------------------*/

#include "planner.h"

//-----------------------------------------------------------------------------

/// Standalone Planner Entry Point
// TODO : refactor to exit as gracefully as possible
int main( int argc, char* argv[] ) {

  // * For Abstract Controller Main *
  // process command line to determine correct controller.
  // create the controller
  // initialize controller
  // start controller

  planner_c planner( argv );
  int cycle = 0;
  notification_c notification;
  simtime_t simtime;
  int write_channel;

  // NOTE: this is now a parameter of the arguments.  pulled from serialization
  const double INTERVAL = 1.0 / (double) CONTROLLER_HZ;

  // NOTE: Subject to parameters.  Will not necessarily initialize at zero.
  simtime.seconds = 0.0;

  // Note: if init fails, the planner bombs out.  Will write a message to
  // console and/or error_log if this happens.
  planner.init( );

  // query and publish initial values
  planner.activate( simtime );

  write_channel = FD_PLANNER_TO_COORDINATOR_WRITE_CHANNEL;

  // build an idle notification.  will be reused each cycle.
  notification.type = NOTIFICATION_IDLE;

  // lock into memory to prevent pagefaults
  mlockall( MCL_CURRENT );

  // get the current timestamp
  notification.ts = generate_timestamp( );

  // start the main process loop
  while( 1 ) {

    // send an idle notification
    if( write( write_channel, &notification, sizeof( notification_c ) ) == -1 ){
        // failed to write, should record in error log?
    }

    // awoken here and now, so get a new timestamp
    notification.ts = generate_timestamp( );

    // Note: may have some long term drift due to fp math
    simtime.seconds += INTERVAL;

    // run the specialty code for this user process
    planner.activate( simtime );

    cycle++;
  }

  munlockall( );

  planner.shutdown( );

  return 0;
}

//-----------------------------------------------------------------------------


