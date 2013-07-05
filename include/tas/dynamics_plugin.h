#ifndef _DYNAMICS_PLUGIN_H_
#define _DYNAMICS_PLUGIN_H_

//-----------------------------------------------------------------------------

typedef double Real;

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
typedef void ( *step_f )( const Real& dt );

// dynamics write state function signature
typedef void ( *write_state_f )( void );

// dynamics read command function signature
typedef void ( *read_command_f )( void );


//-----------------------------------------------------------------------------

class dynamics_plugin_c {
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

    //---------------------------------------------------------------------------

    /**
  given the path to a file (filename), attempts to read dynamics plugin into the set of
  registered plugins.  dynamics plugins must support both init_fun_t and run_fun_t
  interfaces
  */
    plugin_err_e read( const char* filename ) {
        /*
    plugin_err_e result = open( filename );
    if( result != PLUGIN_ERROR_NONE ) return result;

    result = link( DYNAMICS_FUNCTION_INIT );
    if( result != PLUGIN_ERROR_NONE ) return result;

    result = link( DYNAMICS_FUNCTION_RUN );
    if( result != PLUGIN_ERROR_NONE ) return result;

    result = link( DYNAMICS_FUNCTION_READ );
    if( result != PLUGIN_ERROR_NONE ) return result;

    result = link( DYNAMICS_FUNCTION_WRITE );
    return result;
*/
        ///*
        if( open( filename )  )
            // attempt to read the plugin file
            HANDLE = dlopen( filename, RTLD_LAZY );
        if ( !HANDLE ) {
            std::cerr << " failed to read plugin from " << filename << std::endl;
            std::cerr << "  " << dlerror( ) << std::endl;
            return PLUGIN_ERROR_OPEN;
        }

        // locate the init function
        init = (init_f) dlsym(HANDLE, "init");
        const char* dlsym_error1 = dlerror( );
        if ( dlsym_error1 ) {
            std::cerr << " warning: cannot load symbol 'init' from " << filename << std::endl;
            std::cerr << "        error follows: " << std::endl << dlsym_error1 << std::endl;
            return PLUGIN_ERROR_LINK;
        }

        shutdown = (shutdown_f) dlsym(HANDLE, "shutdown");
        const char* dlsym_error2 = dlerror( );
        if ( dlsym_error2 ) {
            std::cerr << " warning: cannot load symbol 'init' from " << filename << std::endl;
            std::cerr << "        error follows: " << std::endl << dlsym_error2 << std::endl;
            return PLUGIN_ERROR_LINK;
        }

        // locate the run function
        step = (step_f) dlsym(HANDLE, "step");
        const char* dlsym_error3 = dlerror( );
        if ( dlsym_error3 ) {
            std::cerr << " warning: cannot load symbol 'step' from " << filename << std::endl;
            std::cerr << "        error follows: " << std::endl << dlsym_error3 << std::endl;
            return PLUGIN_ERROR_LINK;
        }

        // locate the write state function
        write_state = (write_state_f) dlsym(HANDLE, "write_state");
        const char* dlsym_error4 = dlerror( );
        if ( dlsym_error4 ) {
            std::cerr << " warning: cannot load symbol 'write_state' from " << filename << std::endl;
            std::cerr << "        error follows: " << std::endl << dlsym_error4 << std::endl;
            return PLUGIN_ERROR_LINK;
        }

        // locate the read_command function
        read_command = (read_command_f) dlsym(HANDLE, "read_command");
        const char* dlsym_error5 = dlerror( );
        if ( dlsym_error5 ) {
            std::cerr << " warning: cannot load symbol 'read_command' from " << filename << std::endl;
            std::cerr << "        error follows: " << std::endl << dlsym_error5 << std::endl;
            return PLUGIN_ERROR_LINK;
        }
        return PLUGIN_ERROR_NONE;
        //*/
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
        if ( !HANDLE ) {
            //if( VERBOSE ) std::cerr << " failed to open plugin: " << filename << std::endl;
            //if( VERBOSE ) std::cerr << "  " << dlerror( ) << std::endl;
            return PLUGIN_ERROR_OPEN;
        }
        return PLUGIN_ERROR_NONE;
    }

    //---------------------------------------------------------------------------
/*
    plugin_err_e link( dynamics_function_e function ) {
        if( !HANDLE ) return PLUGIN_ERROR_OPEN;

        std::string fname;
        if( function == DYNAMICS_FUNCTION_INIT ) {
            fname = "init";
            init = (init_f) dlsym( HANDLE, fname.c_str( ) );
        } else if( function == DYNAMICS_FUNCTION_SHUTDOWN ) {
            fname = "shutdown";
            shutdown = (shutdown_f) dlsym( HANDLE, fname.c_str( ) );
        } else if( function == DYNAMICS_FUNCTION_RUN ) {
            fname = "step";
            step = (step_f) dlsym( HANDLE, fname.c_str( ) );
        } else if( function == DYNAMICS_FUNCTION_WRITE ) {
            fname = "write_state";
            write_state = (write_state_f) dlsym( HANDLE, fname.c_str( ) );
        } else if( function == DYNAMICS_FUNCTION_READ ) {
            fname = "read_command";
            read_command = (read_command_f) dlsym( HANDLE, fname.c_str( ) );
        } else {
            return PLUGIN_ERROR_LINK;
        }

        const char* dlsym_error = dlerror( );
        if ( dlsym_error ) {
            //if( VERBOSE ) std::cerr << " cannot load symbol '" << fname << std::endl;
            std::cerr << "        error follows: " << std::endl << dlsym_error << std::endl;
            return PLUGIN_ERROR_LINK;
        }
        return PLUGIN_ERROR_NONE;
    }
*/
    //---------------------------------------------------------------------------

};

//-----------------------------------------------------------------------------

#endif // _DYNAMICS_PLUGIN_H_
