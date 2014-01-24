/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

controller.cpp
-----------------------------------------------------------------------------*/

#include "controller.h"

//-----------------------------------------------------------------------------

/// Standalone Controller Entry Point
// TODO : refactor to exit as gracefully as possible
int main( int argc, char* argv[] ) {

  // * For Abstract Controller Main *
  // process command line to determine correct controller.
  // create the controller
  // initialize controller
  // start controller

  controller_c controller(argv);
  int cycle = 0;
  controller_notification_c notification;
  timestamp_t ts;
  const double INTERVAL = 1.0 / (double) CONTROLLER_HZ;

  // Note: if init fails, the controller bombs out.  Will write a message to
  // console and/or error_log if this happens.
  controller.init( );

  // query and publish initial values
  controller.activate( controller.simtime );

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

    int write_channel;
    if( controller.self->PLAYER_TYPE == ship_c::PREDATOR ) {
      write_channel = FD_PREDATOR_TO_COORDINATOR_WRITE_CHANNEL;
    } else {
      write_channel = FD_PREY_TO_COORDINATOR_WRITE_CHANNEL;
    }

    // send a snooze notification
    if( write( write_channel, &notification, sizeof( controller_notification_c ) ) == -1 ) {
        // failed to write, should record in error log?
    }

    //READ? to get the controller to block and to ensure it doesn't overrun?  Ideally, the write event
    // causes the coordinator to wake and interrupt this process so there shouldn't be overrun.

    // unsnoozed here, so get a new timestamp
    ts = generate_timestamp( );
    //rdtscll( ts.cycle );
    // Note: may have some long term drift due to fp math
    controller.simtime.seconds += INTERVAL;

    controller.activate( controller.simtime );

    cycle++;
  }

  munlockall( );

  controller.shutdown( );

  return 0;
}

//-----------------------------------------------------------------------------
