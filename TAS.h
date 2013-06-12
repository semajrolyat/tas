/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

TAS.h
-----------------------------------------------------------------------------*/

#ifndef _TAS_H_
#define _TAS_H_

//-----------------------------------------------------------------------------
/*
// C/C++ includes
#include <assert.h>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>



// POSIX includes
#include <pthread.h>

#include <errno.h>
*/

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
#include <sched.h>
#include <dlfcn.h>
#include <sys/types.h>

#include <sstream>
#include <fstream>
#include <iostream>
#include <list>

#include <stdint.h>

#include <pthread.h>

#include <Moby/Simulator.h>
#include <Moby/RCArticulatedBody.h>

#include "ActuatorMessage.h"
#include "DynamicsPlugin.h"

#include <sys/mman.h>
#include <sys/stat.h>
//#include <sys/wait.h>

//-----------------------------------------------------------------------------

typedef double Real;

#define PI 3.14159265359

#define VERBOSE 1

//-----------------------------------------------------------------------------
// IPC Channels
//-----------------------------------------------------------------------------

#define FD_COORDINATOR_TO_CONTROLLER_READ_CHANNEL   1000
#define FD_COORDINATOR_TO_CONTROLLER_WRITE_CHANNEL  1001

#define FD_CONTROLLER_TO_COORDINATOR_READ_CHANNEL   1002
#define FD_CONTROLLER_TO_COORDINATOR_WRITE_CHANNEL  1003

//-----------------------------------------------------------------------------

#define CONTROLLER_FREQUENCY_HERTZ                  1000

//-----------------------------------------------------------------------------

#define DYNAMICS_PLUGIN "/home/james/tas/build/libdynamics.so"
#define DEFAULT_CONTROLLER_PROGRAM      "controller"

//-----------------------------------------------------------------------------

#endif // _TAS_H_

