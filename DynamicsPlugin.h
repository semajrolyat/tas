#ifndef _DYNAMICSPLUGIN_H_
#define _DYNAMICSPLUGIN_H_

//-----------------------------------------------------------------------------

typedef double Real;

//-----------------------------------------------------------------------------

// dynamics initialization function signature
typedef void (*init_fun_t)( int argv, char** argc );

// dynamics run function signature
typedef void (*run_fun_t)( Real dt );

// dynamics publish state function signature
typedef void (*pub_state_fun_t)( void );

// dynamics get command function signature
typedef void (*get_command_fun_t)( void );


//-----------------------------------------------------------------------------

class DynamicsPlugin {
public:
  DynamicsPlugin( void ) { HANDLE = NULL; }
  virtual ~DynamicsPlugin( void ) { }

  init_fun_t          init;
  run_fun_t           run;
  pub_state_fun_t     pubstate;
  get_command_fun_t   getcommand;

  // plugin handle
  void* HANDLE;

  /**
  given the path to a file (filename), attempts to read dynamics plugin into the set of
  registered plugins.  dynamics plugins must support both init_fun_t and run_fun_t
  interfaces
  */
  int read( const char* filename ) {
    // attempt to read the plugin file
    HANDLE = dlopen( filename, RTLD_LAZY );
    if ( !HANDLE ) {
      std::cerr << " failed to read plugin from " << filename << std::endl;
      std::cerr << "  " << dlerror( ) << std::endl;
      return 1;
    }

    // locate the init function
    bool failed = false;
    init = (init_fun_t) dlsym(HANDLE, "init");
    const char* dlsym_error1 = dlerror( );
    if ( dlsym_error1 ) {
      std::cerr << " warning: cannot load symbol 'init' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error1 << std::endl;
      failed = true;
      return 2;
    }

    // locate the run function
    run = (run_fun_t) dlsym(HANDLE, "run");
    const char* dlsym_error2 = dlerror( );
    if ( dlsym_error2 ) {
      std::cerr << " warning: cannot load symbol 'run' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error2 << std::endl;
      failed = true;
      return 3;
    }

    // locate the publish state function
    pubstate = (pub_state_fun_t) dlsym(HANDLE, "publish_state");
    const char* dlsym_error3 = dlerror( );
    if ( dlsym_error3 ) {
      std::cerr << " warning: cannot load symbol 'publish_state' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error3 << std::endl;
      failed = true;
      return 4;
    }

    // locate the get_command function
    getcommand = (get_command_fun_t) dlsym(HANDLE, "get_command");
    const char* dlsym_error4 = dlerror( );
    if ( dlsym_error4 ) {
      std::cerr << " warning: cannot load symbol 'get_command' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error4 << std::endl;
      failed = true;
      return 5;
    }
    return 0;
  }

  void close( void ) {
    if( HANDLE != NULL )
        dlclose( HANDLE );
  }
};

//-----------------------------------------------------------------------------

#endif // _DYNAMICSPLUGIN_H_
