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
  controller_notification_c notification;
  timestamp_t ts;
  const double INTERVAL = 1.0 / (double) CONTROLLER_HZ;
  simtime_t simtime;

  simtime.seconds = 0.0;

  // Note: if init fails, the controller bombs out.  Will write a message to
  // console and/or error_log if this happens.
  planner.init( );

  // query and publish initial values
  planner.activate( simtime );

  notification.type = CONTROLLER_NOTIFICATION_SNOOZE;
  notification.duration = INTERVAL;

  // lock into memory to prevent pagefaults
  mlockall( MCL_CURRENT );

  // get the current timestamp
  ts = generate_timestamp( );
  //rdtscll( ts.cycle );

  // start the main process loop
  while( 1 ) {

    notification.ts = ts;

    // send a snooze notification
    if( write( FD_PLANNER_TO_COORDINATOR_WRITE_CHANNEL, &notification, sizeof( controller_notification_c ) ) == -1 ) {
        // failed to write, should record in error log?
    }

    //READ? to get the controller to block and to ensure it doesn't overrun?  Ideally, the write event
    // causes the coordinator to wake and interrupt this process so there shouldn't be overrun.

    // unsnoozed here, so get a new timestamp
    ts = generate_timestamp( );
    //rdtscll( ts.cycle );
    // Note: may have some long term drift due to fp math
    simtime.seconds += INTERVAL;

    planner.activate( simtime );

    cycle++;
  }

  munlockall( );

  planner.shutdown( );

  return 0;
}

//-----------------------------------------------------------------------------


