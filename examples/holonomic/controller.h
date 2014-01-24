#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

//-----------------------------------------------------------------------------
/*
#include "common.h"
#include "constants.h"
#include "aabb.h"
#include "command.h"
#include "state.h"
#include "utilities.h"
#include "space.h"
*/

#include "memory.h"
#include "message.h"
//-----------------------------------------------------------------------------

#include <assert.h>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>

#include <Moby/Simulator.h>
#include <Moby/RigidBody.h>

#include "tas.h"
#include "time.h"
#include "log.h"
#include "cpu.h"
#include "experiment.h"
#include "ipc.h"

#include "error.h"

#include "ship.h"
#include "space.h"

#include "thread.h"

//-----------------------------------------------------------------------------

class controller_c : public thread_c {
public:
  double FEEDBACK_GAIN_PROPORTIONAL_POSITION;
  double FEEDBACK_GAIN_DERIVATIVE_POSITION;
  double FEEDBACK_GAIN_PROPORTIONAL_ROTATION;
  double FEEDBACK_GAIN_DERIVATIVE_ROTATION;

  ship_p self;
  ship_p adversary;

  space_p space;

  double DT;
  simtime_t simtime;

  log_c error_log;
  char err_buffer[256];
  sharedbuffer_c msg_buffer;
  cpuinfo_c cpuinfo;
  unsigned long long cpu_speed_hz;

  controller_c( void );
  controller_c( char* argv[] );
  virtual ~controller_c( void );

  virtual void shutdown( void );
  virtual void init( void );
  virtual error_e control( const act_msg_c& in, act_msg_c& out );
  virtual error_e request( const simtime_t& time, act_msg_c& state );
  virtual error_e publish( const act_msg_c& command );
  virtual error_e activate( const simtime_t& time );

  ship_command_c compute_feedback( const ship_state_c& q, ship_command_c& u, const double& Kp_position, const double& Kd_position, const double& Kp_rotation, const double& Kd_rotation );
  void compute_desired_state( const ship_state_c& q0, const ship_command_c& u, ship_state_c& q, const double& dt );


  virtual void execute( void );
}; 

//-----------------------------------------------------------------------------

#endif // _CONTROLLER_H_

