/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

Extension of ut_controllerschedule.cpp prototyping.  This test validates
interprocess communication and process manangement over an external controller
program, schedule handling of the controller process and accurate time
slicing of the controller process.
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
#include <sched.h>
#include <dlfcn.h>
#include <sys/types.h>

#include <sstream>
#include <fstream>
#include <iostream>
#include <list>

#include <stdint.h>

#include <Moby/Simulator.h>
#include <Moby/RCArticulatedBody.h>

#include "ActuatorMessage.h"
#include "DynamicsPlugin.h"

//-----------------------------------------------------------------------------







