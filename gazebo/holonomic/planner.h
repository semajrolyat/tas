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
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multifit_nlin.h>

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

class planner_c {
public:
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

  // the reference to the statespace the planner has generated
  ompl::base::StateSpace *statespace;
  ompl::control::SpaceInformationPtr si;

  ode_f ode;
  best_control_f best_control;
  prey_command_f prey_command;

  planner_c( void ) {}
  planner_c( const planner_type_e& type, ode_f _ode, best_control_f _bc, prey_command_f _pc, aabb_c _spatial_bound, ship_p _self, ship_p _adversary, const double& step_size, const double& max_plan_time, const double& max_derivative, const double& max_force, const double& goal_bias ) {
    ode = _ode;
    best_control = _bc;
    prey_command = _pc;
    spatial_bound = _spatial_bound;
    self = _self;
    adversary = _adversary;

    PLANNER_STEP_SIZE = step_size;
    PLANNER_MAX_PLANNING_TIME = max_plan_time;
    PLANNER_MAX_DERIVATIVE = max_derivative;
    PLANNER_MAX_FORCE = max_force;
    PLANNER_GOAL_BIAS = goal_bias;
  }
  virtual ~planner_c( void ) {}


  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  bool plan_for_prey( const double& t, pp_state_c& q, ship_command_c& u ) {
    std::vector<double> pred_q, prey_q, prey_u;

    pred_q = q.pred_vector();
    prey_q = q.prey_vector();

    prey_command( pred_q, prey_q, prey_u, adversary, self, t );

    u = ship_command_c( prey_u );

    return true;
  }

  //---------------------------------------------------------------------------
  bool plan_for_predator( const double& t, pp_state_c& q, ship_command_c& u ) {

    // x, y, z extens
    Ravelin::Vector3d x( spatial_bound.extens.x(),
                         spatial_bound.extens.y(),
                         spatial_bound.extens.z() );
    // quaternion extens 
    Ravelin::Quatd e( 1.0, 1.0, 1.0, 1.0 );
    // change in state
    Ravelin::Vector3d dx( PLANNER_MAX_DERIVATIVE,
                          PLANNER_MAX_DERIVATIVE,
                          PLANNER_MAX_DERIVATIVE );
    Ravelin::Vector3d de( PLANNER_MAX_DERIVATIVE,
                          PLANNER_MAX_DERIVATIVE,
                          PLANNER_MAX_DERIVATIVE );

    ship_state_c qd( x, e, dx, de );
    ship_state_c qy( x, e, dx, de );

    pp_state_c q_bounds( qd, qy );
    ship_command_c u_bounds;

    for( unsigned i = 0; i < ship_command_c::size(); i++ )
      u_bounds(i) = PLANNER_MAX_FORCE;

    return plan_rrt( self, adversary, q, q_bounds, u, u_bounds );
  }

  //----------------------------------------------------------------------------
  /// Allocates a predator/prey scenario control sampler
  ompl::control::DirectedControlSamplerPtr allocate_control_sampler( const ompl::control::SpaceInformation* si ) {
    int k = 1;
    //return ompl::control::DirectedControlSamplerPtr( new control_sampler_c( si, self, adversary, &ship_c::best_control, k ) );
    return ompl::control::DirectedControlSamplerPtr( new control_sampler_c( si, self, adversary, best_control, k ) );
  }


  //---------------------------------------------------------------------------
  // OMPL Planning
  //---------------------------------------------------------------------------
  /// Plans motion for the ship using an rrt planner
  bool plan_rrt( ship_p self, ship_p adversary, pp_state_c& q, const pp_state_c& q_bounds, ship_command_c& u, const ship_command_c& u_bounds ) {

    ompl::base::StateSpacePtr sspace( new ompl::base::RealVectorStateSpace( pp_state_c::size() ) );
    statespace = sspace.get();

    // Define bounds for the state space
    ompl::base::RealVectorBounds bounds( pp_state_c::size() );

    for( unsigned i = 0; i < pp_state_c::size(); i++ ) {
      bounds.setLow( i, -q_bounds.value(i) );
      bounds.setHigh( i, q_bounds.value(i) );
    }
    sspace->as<ompl::base::RealVectorStateSpace>()->setBounds( bounds );

    // create a control space
    ompl::control::ControlSpacePtr cspace( new control_space_c( sspace ) );

    // set the bounds for the control space
    ompl::base::RealVectorBounds cbounds( ship_command_c::size() );
    for( unsigned i = 0; i < ship_command_c::size(); i++ )  {
      cbounds.setLow( i, -u_bounds.value(i) );
      cbounds.setHigh( i, u_bounds.value(i) );
    }
    cspace->as<control_space_c>()->setBounds( cbounds );

    // define a simple setup class
    ompl::control::SimpleSetup ss( cspace );

    ompl::control::SpaceInformationPtr si( ss.getSpaceInformation() );
    this->si = si;
    //si = ompl::control::SpaceInformationPtr( ss.getSpaceInformation() );

    // !
    si->setDirectedControlSamplerAllocator( boost::bind(&planner_c::allocate_control_sampler, this, _1) );

    ompl::base::ProblemDefinition pdef( si );

    // !
    /// set state validity checking for this space
    //ss.setStateValidityChecker( boost::bind(&ship_c::is_state_valid, self.get(), si.get(), _2 ) );
    ss.setStateValidityChecker( boost::bind( &planner_c::is_state_valid, this, si.get(), _1 ) );
    //ss.setStateValidityChecker( &is_state_valid );

    // !
    /// set the propagation routine for this space
    ss.setStatePropagator( ompl::control::StatePropagatorPtr( new state_propagator_c( si, self, adversary, ode, prey_command ) ) );

    /// create a start state
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start( sspace );
  q.write_ompl_state( statespace, start.get() );

    /// create a goal
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal( sspace );
    boost::shared_ptr <pp_goal_c> pgoal( new pp_goal_c( si ) );

    /// set the start and goal
    ss.setStartState( start );
    ss.setGoal( pgoal );

    ompl::control::RRT* rrt = new ompl::control::RRT(si);
    rrt->setGoalBias( PLANNER_GOAL_BIAS );
    ompl::base::PlannerPtr p( rrt );
    ss.setPlanner( p );

    ss.getSpaceInformation()->setPropagationStepSize( PLANNER_STEP_SIZE );
    //ss.getSpaceInformation()->setMinMaxControlDuration(1, 10);
    ss.setup();
    static_cast<state_propagator_c*>(ss.getStatePropagator().get())->setIntegrationTimeStep(ss.getSpaceInformation()->getPropagationStepSize());

    ompl::base::PlannerStatus solved;
    solved = ss.solve( PLANNER_MAX_PLANNING_TIME );

    if( solved ) {
      std::cout << "Found solution:" << std::endl;
      u = ship_command_c( ss.getSolutionPath().getControl(0) );
      return true;
    } else {
      std::cout << "No solution found" << std::endl;
      return false;
    }
  }

  //---------------------------------------------------------------------------
  /// Determines whether the a state is a valid configuration for a ship
  bool is_state_valid(const ompl::control::SpaceInformation *si, const ompl::base::State *state) {

  //TODO : this has to be refactored for querying boundary information from si and not from in class
/*
  std::vector<double> values( ship_state_c::size() );
  from_state( si->getStateSpace().get(), state, values );
 
  aabb_c bb = aabb( values );
  aabb_c obstacle;
  
  if( intersects_world_bounds( bb ) )
    return false;
  if( intersects_any_obstacle( bb, obstacle ) )
    return false;
*/
  return true;
}

};

//-----------------------------------------------------------------------------

#endif // _PLANNER_H_

