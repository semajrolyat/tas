#ifndef _DYNAMICS_H_
#define _DYNAMICS_H_

#include "thread.h"

//-----------------------------------------------------------------------------

enum plugin_err_e {
  PLUGIN_ERROR_NONE,
  PLUGIN_ERROR_OPEN,
  PLUGIN_ERROR_LINK,
  PLUGIN_ERROR_READ
};

//-----------------------------------------------------------------------------

enum dynamics_function_e {
  DYNAMICS_FUNCTION_INIT,
  DYNAMICS_FUNCTION_SHUTDOWN,
  DYNAMICS_FUNCTION_RUN,
  DYNAMICS_FUNCTION_READ,
  DYNAMICS_FUNCTION_WRITE
};
//-----------------------------------------------------------------------------

// dynamics initialization function signature
typedef void ( *init_f )( int argv, char** argc );

// dynamics initialization function signature
typedef void ( *shutdown_f )( void );

// dynamics run function signature
typedef void ( *step_f )( const double& dt );

// dynamics write state function signature
typedef void ( *write_state_f )( void );

// dynamics read command function signature
typedef void ( *read_command_f )( void );


//-----------------------------------------------------------------------------

class dynamics_plugin_c : public thread_c {
private:

  void* HANDLE; 	// plugin handle

  //---------------------------------------------------------------------------

public:
  dynamics_plugin_c( void ) { HANDLE = NULL; }
  virtual ~dynamics_plugin_c( void ) { }

  init_f          init;
  shutdown_f      shutdown;
  step_f          step;
  write_state_f   write_state;
  read_command_f  read_command;

  double step_size;

  //---------------------------------------------------------------------------
  virtual void execute( void ) {
    (*step)(step_size);
  }

  //---------------------------------------------------------------------------
  virtual void suspend( void ) {

  }

  //---------------------------------------------------------------------------
  virtual void resume( void ) {

  }
/*
  //---------------------------------------------------------------------------
  // Note: that this method is provided by the plugin interface through a 
  // function pointer.  However, anticipate a linking or segmentation issue
  // due to inheritence of the method.
  virtual void shutdown( void ) {

  }
*/
  //---------------------------------------------------------------------------
  plugin_err_e read( const char* filename ) {
    if( open( filename )  )
      // attempt to read the plugin file
      HANDLE = dlopen( filename, RTLD_LAZY );
    if( !HANDLE ) {
      std::cerr << " failed to read plugin from " << filename << std::endl;
      std::cerr << "  " << dlerror( ) << std::endl;
      return PLUGIN_ERROR_OPEN;
    }

    // locate the init function
    init = (init_f) dlsym(HANDLE, "init");
    const char* dlsym_error1 = dlerror( );
    if( dlsym_error1 ) {
      std::cerr << " warning: cannot load symbol 'init' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error1 << std::endl;
      return PLUGIN_ERROR_LINK;
    }

    shutdown = (shutdown_f) dlsym(HANDLE, "shutdown");
    const char* dlsym_error2 = dlerror( );
    if( dlsym_error2 ) {
      std::cerr << " warning: cannot load symbol 'init' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error2 << std::endl;
      return PLUGIN_ERROR_LINK;
    }

    // locate the run function
    step = (step_f) dlsym(HANDLE, "step");
    const char* dlsym_error3 = dlerror( );
    if( dlsym_error3 ) {
      std::cerr << " warning: cannot load symbol 'step' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error3 << std::endl;
      return PLUGIN_ERROR_LINK;
    }

    // locate the write state function
    write_state = (write_state_f) dlsym(HANDLE, "write_state");
    const char* dlsym_error4 = dlerror( );
    if( dlsym_error4 ) {
      std::cerr << " warning: cannot load symbol 'write_state' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error4 << std::endl;
      return PLUGIN_ERROR_LINK;
    }

    // locate the read_command function
    read_command = (read_command_f) dlsym(HANDLE, "read_command");
    const char* dlsym_error5 = dlerror( );
    if( dlsym_error5 ) {
      std::cerr << " warning: cannot load symbol 'read_command' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error5 << std::endl;
      return PLUGIN_ERROR_LINK;
    }
    return PLUGIN_ERROR_NONE;
  }
  //---------------------------------------------------------------------------
  void close( void ) {
    if( HANDLE != NULL )
      dlclose( HANDLE );
  }
  //---------------------------------------------------------------------------

private:
  //---------------------------------------------------------------------------
  plugin_err_e open( const char* filename ) {

    HANDLE = dlopen( filename, RTLD_LAZY );
    if( !HANDLE ) {
      //if( VERBOSE ) std::cerr << " failed to open plugin: " << filename << std::endl;
      //if( VERBOSE ) std::cerr << "  " << dlerror( ) << std::endl;
      return PLUGIN_ERROR_OPEN;
    }
    return PLUGIN_ERROR_NONE;
  }

  //---------------------------------------------------------------------------

};

//-----------------------------------------------------------------------------

#endif // _DYNAMICS_H_
