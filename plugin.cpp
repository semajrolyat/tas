/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

Prototype of a Plugin for the Time Accurate Simulator system.
-----------------------------------------------------------------------------*/

#include <stdio.h>
#include <iostream>
#include <dlfcn.h>

#include <Moby/Vector3.h>
#include <Moby/RigidBody.h>

// Plugins have to extern "C"
extern "C" {

using namespace Moby;

/// Handle for dynamic library loading
void* HANDLE = NULL;

/// The default simulation step size
const Real DEFAULT_STEP_SIZE = .001;

/// The simulation step size
Real STEP_SIZE = DEFAULT_STEP_SIZE;

/// The map of objects read from the simulation XML file
std::map<std::string, BasePtr> READ_MAP;

/// Pointer to the controller's initializer, called once (if any)
typedef void (*init_t)(void*, const std::map<std::string, BasePtr>&, Real);
std::list<init_t> INIT;

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

// attempts to read control code plugin
void read_plugin(const char* filename)
{
  // attempt to read the file
  HANDLE = dlopen(filename, RTLD_LAZY);
  if (!HANDLE)
  {
    std::cerr << "driver: failed to read plugin from " << filename << std::endl;
    std::cerr << "  " << dlerror() << std::endl;
    exit(-1);
  }

  // attempt to load the initializer
  dlerror();
  INIT.push_back((init_t) dlsym(HANDLE, "init"));
  const char* dlsym_error = dlerror();
  if (dlsym_error)
  {
    std::cerr << "driver warning: cannot load symbol 'init' from " << filename << std::endl;
    std::cerr << "        error follows: " << std::endl << dlsym_error << std::endl;
    INIT.pop_back();
  }
}

void initialize_plugins( ) {
    for( std::list<init_t>::iterator it = INIT.begin(); it != INIT.end(); it++ ) {
        (*it)(NULL, READ_MAP, STEP_SIZE);
    }
}


//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

// Entry Point - defined by plugin interface exposed by TAS system
void init( int argc, char** argv ) {
    printf( "Hello Plugin!\n" );

    //Vector3 x_hat = Vector3( 1.0, 0.0, 0.0 );

    //printf( "x_hat ( %f, %f, %f )\n", x_hat[0], x_hat[1], x_hat[2] );

    char* filename = "/home/james/Moby/build/libempty-plugin.so";

    read_plugin( filename );

    initialize_plugins( );
}

} // close extern "C"
