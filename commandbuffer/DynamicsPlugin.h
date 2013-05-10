#ifndef _DYNAMICSPLUGIN_H_
#define _DYNAMICSPLUGIN_H_

// dynamics initialization function signature
typedef void (*init_fun_t)( int argv, char** argc );

// dynamics run function signature
typedef void (*run_fun_t)( void );


typedef void (*pub_state_fun_t)( void );

typedef void (*get_cmd_fun_t)( void );


class DynamicsPlugin {
public:
    DynamicsPlugin( void ) { }
    virtual ~DynamicsPlugin( void ) { }

    init_fun_t          init;
    run_fun_t           run;
    pub_state_fun_t     pubstate;
    get_cmd_fun_t       getcmd;

};

#endif // _DYNAMICSPLUGIN_H_
