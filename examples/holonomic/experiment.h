#ifndef _EXPERIMENT_H_
#define _EXPERIMENT_H_

#define ERRCORRECT

//-----------------------------------------------------------------------------
// SIMULATION
//-----------------------------------------------------------------------------

#define SIM_DURATION_CYCLES 100
//#define SIM_DURATION_CYCLES 1000
//#define SIM_DURATION_CYCLES 10000
//#define SIM_DURATION_CYCLES 100000

//-----------------------------------------------------------------------------
// PROCESSOR
//-----------------------------------------------------------------------------

#define DEFAULT_PROCESSOR    0

//-----------------------------------------------------------------------------
// REALTIME
//-----------------------------------------------------------------------------

#define REALTIME

//-----------------------------------------------------------------------------
// CONTROLLER
//-----------------------------------------------------------------------------

//#define CONTROLLER_HZ                  1000
#define CONTROLLER_HZ                  100
#define CONTROLLER_PROGRAM      "controller"

//-----------------------------------------------------------------------------
// DYNAMICS PLUGIN
//-----------------------------------------------------------------------------

#define DYNAMICS_PLUGIN "/home/james/tas/build/libdynamics.so"

//-----------------------------------------------------------------------------

#endif // _EXPERIMENT_H_
