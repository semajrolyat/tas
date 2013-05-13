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
    DynamicsPlugin( void ) { }
    virtual ~DynamicsPlugin( void ) { }

    init_fun_t          init;
    run_fun_t           run;
    pub_state_fun_t     pubstate;
    get_command_fun_t   getcommand;
};

//-----------------------------------------------------------------------------

#endif // _DYNAMICSPLUGIN_H_
