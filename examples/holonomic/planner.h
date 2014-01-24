#ifndef _PLANNER_H_
#define _PLANNER_H_

//-----------------------------------------------------------------------------
#include "common.h"
#include "constants.h"
#include "aabb.h"
#include "goal.h"
#include "control_sampler.h"
#include "state_propagator.h"
#include "control_space.h"

//-----------------------------------------------------------------------------
// Planner
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

//-----------------------------------------------------------------------------
// PDF
#ifdef GAZEBO_DYNAMICS
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multifit_nlin.h>
#endif
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
#include "common.h"
#include "constants.h"
#include "aabb.h"
#include "command.h"
#include "state.h"
#include "utilities.h"
#include "space.h"

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
//--
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
/*
class planner_c {
public:
  log_c error_log;
  char strbuffer[256];
  sharedbuffer_c amsgbuffer;
  cpuinfo_c cpuinfo;
  unsigned long long cpu_speed_hz;

  planner_c( void );
  virtual ~planner_c( void );

  virtual void shutdown( void );
  virtual void init( void );
  virtual error_e plan( const message_c& in, message_c& out );
  virtual error_e request( const double& time, message_c& state );
  virtual error_e publish( const message_c& command );
  virtual error_e step( const double& time );
}; 
*/
class planner_c : public thread_c {
public:
  log_c error_log;
  char err_buffer[256];
  sharedbuffer_c msg_buffer;
  cpuinfo_c cpuinfo;
  unsigned long long cpu_speed_hz;

  enum planner_type_e {
    WALK,
    RRT
  };

  double PLANNER_STEP_SIZE;
  double PLANNER_MAX_PLANNING_TIME;
  double PLANNER_MAX_DERIVATIVE;
  double PLANNER_MAX_FORCE;
  double PLANNER_GOAL_BIAS;

  planner_type_e type;

  aabb_c spatial_bound;

  ship_p self;
  ship_p adversary;
  space_p space;

  // the reference to the statespace the planner has generated
  ompl::base::StateSpace *statespace;
  ompl::control::SpaceInformationPtr si;

  ode_f ode;
  best_control_f best_control;
  prey_command_f prey_command;

  planner_c( void );
  planner_c( char* argv[] );
  planner_c( const planner_type_e& type, ode_f _ode, best_control_f _bc, prey_command_f _pc, aabb_c _spatial_bound, ship_p _self, ship_p _adversary, const double& step_size, const double& max_plan_time, const double& max_derivative, const double& max_force, const double& goal_bias );
  
  virtual ~planner_c( void );

  virtual void shutdown( void );
  virtual void init( void );
  virtual error_e plan( const act_msg_c& in, act_msg_c& out );
  virtual error_e request( const simtime_t& time, act_msg_c& state );
  virtual error_e publish( const act_msg_c& command );
  virtual error_e activate( const simtime_t& time );

  //---------------------------------------------------------------------------
  //bool plan_for_prey( const double& t, const pp_state_c& q, ship_command_c& u );
  bool plan_for_predator( const double& t, const pp_state_c& q, ship_command_c& u );

  /// Plans motion for the ship using an rrt planner
  bool plan_rrt( ship_p self, ship_p adversary, const pp_state_c& q, const pp_state_c& q_bounds, ship_command_c& u, const ship_command_c& u_bounds );

  /// Determines whether the a state is a valid configuration for a ship
  bool is_state_valid(const ompl::control::SpaceInformation *si, const ompl::base::State *state);

  /// Allocates a predator/prey scenario control sampler
  ompl::control::DirectedControlSamplerPtr allocate_control_sampler( const ompl::control::SpaceInformation* si );

  virtual void execute( void );
};

//-----------------------------------------------------------------------------

#endif // _PLANNER_H_

