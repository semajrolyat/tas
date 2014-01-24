#ifndef _CP_H_
#define _CP_H_

//-----------------------------------------------------------------------------
#include "common.h"
#include "constants.h"
#include "aabb.h"
#include "command.h"
#include "state.h"
#include "utilities.h"
#include "space.h"

#include "memory.h"
#include "message.h"
//-----------------------------------------------------------------------------

#include <assert.h>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>

#include <Moby/Simulator.h>
#include <Moby/RigidBody.h>

#include "tas.h"
#include "time.h"
#include "log.h"
#include "cpu.h"
#include "experiment.h"
#include "ipc.h"

#include "error.h"

//-----------------------------------------------------------------------------

class controlledprocess_c {
public:
  log_c error_log;
  char err_buffer[256];
  cpuinfo_c cpuinfo;
  unsigned long long cpu_speed_hz;

private:
  sharedbuffer_c msg_buffer;

public:
  controlledprocess_c( void );
  virtual ~controlledprocess_c( void );

  virtual error_e activate( const message_c& in, message_c& out ) = 0;
  virtual message_c build_request( const double& time ) = 0;

  //---------------------------------------------------------------------------
  virtual void init( void ) {
/*
    // connect to the error log
    // Note: error log is created by the coordinator
    error_log = log_c( FD_ERROR_LOG );
    if( error_log.open( ) != LOG_ERROR_NONE ) {
        // Note:  this is not really necessary.  If coordinator launched properly, this should never happen
        printf( "(cp.cpp) could not open error log. ERROR: main() failed calling log_c.open() on FD_ERROR_LOG\nController Exiting\n" );
        exit( 1 );
    }

    // connect to the shared message buffer BEFORE attempting any IPC
    // Note: the message buffer is created by the coordinator before the controller
    // is launched
    message_c msg;
    msg_buffer = sharedbuffer_c( msg, BUFFER_NAME, false );
    if( msg_buffer.open( ) != BUFFER_ERR_NONE ) {
        sprintf( err_buffer, "(cp.cpp) init() failed calling buffer.open(...,false)\n" );
        printf( "%s", err_buffer );

        error_log.write( err_buffer );
        error_log.close( );
        exit( 1 );
    }
*/
  }

  //---------------------------------------------------------------------------
  virtual void shutdown( void ) {
    msg_buffer.close( );  // TODO : figure out a way to force a clean up
  }

  //---------------------------------------------------------------------------
  virtual error_e request( const double& time, message_c& in ) {

    char buf = 0;
    //message_c request;
    //controller_notification_c notification;
    notification_c notification;

/*
    // build the request message
    in.header.type = MSG_REQUEST;
    request.header.sim_time = time;
*/
    message_c request = build_request( time );


    // write the request to the buffer
    if( msg_buffer.write( in ) != BUFFER_ERR_NONE ) {
        sprintf( err_buffer, "(cp.cpp) request(time,in) failed calling buffer.write(in)\n" );
        error_log.write( err_buffer );
        return ERROR_FAILED;
    }

    // build a notification that a message has been published
    //notification.type = CONTROLLER_NOTIFICATION_ACTUATOR_EVENT;
    //notification.duration = 0.0;
    //rdtscll( notification.ts );

   notification.type = NOTIFICATION_REQUEST;
   notification.ts = generate_timestamp();

    // send the notification
    if( write( FD_CONTROLLER_TO_COORDINATOR_WRITE_CHANNEL, &notification, sizeof(controller_notification_c) ) == -1 ) {
        std::string err_msg = "(cp.cpp) request(time,in) failed writing notification";
        error_log.write( error_string_bad_write( err_msg , errno ) );
        return ERROR_FAILED;
    }

    // wait for reply meaning the request is fulfilled and ready in the buffer
    // Note: controller blocks here on read
    if( read( FD_COORDINATOR_TO_CONTROLLER_READ_CHANNEL, &buf, 1 ) == -1 ) {
        std::string err_msg = "(cp.cpp) request(time,in) failed reading notification";
        error_log.write( error_string_bad_read( err_msg , errno ) );
        return ERROR_FAILED;
    }

    // read the state out of the buffer
    // Note: will block waiting to acquire mutex
    if( msg_buffer.read( in ) != BUFFER_ERR_NONE ) {
        sprintf( err_buffer, "(cp.cpp) request(time,in) failed calling buffer.read(in)\n" );
        error_log.write( err_buffer );
        return ERROR_FAILED;
    }

    return ERROR_NONE;
  }

  //---------------------------------------------------------------------------
  virtual error_e publish( const message_c& out ) {

    controller_notification_c notification;

    // send the message to the buffer
    // Note: will block waiting to acquire mutex
    if( msg_buffer.write( out ) != BUFFER_ERR_NONE) {
        sprintf( err_buffer, "(cp.cpp) publish(out) failed calling buffer.write(out)\n" );
        error_log.write( err_buffer );
        return ERROR_FAILED;
    }

    notification.type = CONTROLLER_NOTIFICATION_ACTUATOR_EVENT;
    notification.duration = 0.0;
    rdtscll( notification.ts );

    // send a notification to the coordinator
    if( write( FD_CONTROLLER_TO_COORDINATOR_WRITE_CHANNEL, &notification, sizeof(controller_notification_c) ) == -1 ) {
        std::string err_msg = "(cp.cpp) publish(out) failed writing notification";
        error_log.write( error_string_bad_write( err_msg , errno ) );
        return ERROR_FAILED;
    }

    return ERROR_NONE;
  }

  //---------------------------------------------------------------------------
  virtual error_e step( const double& time ) {
/*
    message_c in, out;

    // request state
    if( request( time, in ) != ERROR_NONE ) {
        sprintf( err_buffer, "(cp.cpp) step() failed calling request(%f,in)\n", time );
        error_log.write( err_buffer );
        return ERROR_FAILED;
    }

    // activate on the in message from the system and compute out
    if( activate( in, out ) != ERROR_NONE ) {
        sprintf( err_buffer, "(cp.cpp) step() failed calling activate(in,out)\n" );
        error_log.write( err_buffer );
        return ERROR_FAILED;
    }

    // publish the out result
    if( publish( out ) != ERROR_NONE ) {
        sprintf( err_buffer, "(cp.cpp) step() failed calling publish(out)\n" );
        error_log.write( err_buffer );
        return ERROR_FAILED;
    }
*/
    return ERROR_NONE;
  }

}; 

//-----------------------------------------------------------------------------

#endif // _CP_H_

