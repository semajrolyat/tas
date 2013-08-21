/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

tas.h
-----------------------------------------------------------------------------*/

#ifndef _TAS_H_
#define _TAS_H_

//-----------------------------------------------------------------------------

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>             //POSIX
#include <fcntl.h>              //POSIX
#include <sys/time.h>           //POSIX
#include <sys/resource.h>       //POSIX
#include <sched.h>              //POSIX
#include <dlfcn.h>              //POSIX
#include <sys/types.h>          //POSIX
#include <sys/mman.h>           //POSIX
#include <sys/stat.h>           //POSIX
//#include <sys/wait.h>           //POSIX

#include <pthread.h>            //POSIX

// if Moby dynamics
#include <Moby/Simulator.h>
#include <Moby/RCArticulatedBody.h>

#include <tas/time.h>

//-----------------------------------------------------------------------------

typedef double Real;

#define PI 3.14159265359


//-----------------------------------------------------------------------------
// NOTIFICATIONS
//-----------------------------------------------------------------------------

enum controller_notification_type_e {
    CONTROLLER_NOTIFICATION_UNDEFINED = 0,
    CONTROLLER_NOTIFICATION_SNOOZE,
    CONTROLLER_NOTIFICATION_ACTUATOR_EVENT
};

//-----------------------------------------------------------------------------

class controller_notification_c {
public:
    controller_notification_c( void ) {
        type = CONTROLLER_NOTIFICATION_UNDEFINED;
        ts.cycle = 0;
        duration = 0.0;
    }

    virtual ~controller_notification_c( void ) { }

    controller_notification_type_e type;
    timestamp_t ts;
    double duration;
};

//-----------------------------------------------------------------------------

#endif // _TAS_H_

