#ifndef _GAZEBO_SHIP_H_
#define _GAZEBO_SHIP_H_

#include "aabb.h"
#include "command.h"
#include "state.h"
#include "utilities.h"

//-----------------------------------------------------------------------------
#include <ostream>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>
#include <valarray>
#include <limits>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <ompl/control/ODESolver.h>

//#include <stdlib.h>
//#include <stdio.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
//#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlin.h>

#include <Ravelin/Pose3d.h>
#include <Ravelin/SVelocityd.h>
#include <Ravelin/SAcceld.h>
#include <Ravelin/SForced.h>
#include <Ravelin/SpatialRBInertiad.h>
#include <Ravelin/MatrixNd.h>

//-----------------------------------------------------------------------------

typedef double Real;

#define PI 3.14159265359

//-----------------------------------------------------------------------------
// Globals
//-----------------------------------------------------------------------------

class ship_c;           // forward declaration
ship_c* ship;           // a pointer to the ship

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
/// Class encapsulating the ship itself.  A Gazebo plugin
class ship_c : public gazebo::ModelPlugin {
public:

  gazebo::event::ConnectionPtr updateConnection;

  gazebo::physics::ModelPtr model;
  gazebo::physics::WorldPtr world;
  gazebo::physics::LinkPtr body;

  Ravelin::SpatialRBInertiad _inertial;

  double time, dtime, time_start, time_last;

  double GAUSSIAN_MEAN;
  double GAUSSIAN_VARIANCE;
  double GAUSSIAN_STDDEV;

  bool stopped;

  aabb_c ship_frame_bb;
  aabb_c spatial_bound;
  aabb_list_t obstacles;

protected:
  enum planner_type_e {
    WALK,
    RRT
  };

  planner_type_e PLANNER;
public:
  double PLANNER_STEP_SIZE;

  //---------------------------------------------------------------------------
  // OMPL
  //---------------------------------------------------------------------------
public:
  const ompl::base::StateSpace *space_;
  //const ompl::control::SpaceInformationPtr si_;
public:
  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
  ship_c( void );
  ship_c( const ompl::base::StateSpace *space );
    
  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~ship_c( void );

protected:
  //---------------------------------------------------------------------------
  // Gazebo ModelPlugin Interface
  //---------------------------------------------------------------------------
  virtual void Load( gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf );
  virtual void Update( );
  //virtual void Reset( );

  //---------------------------------------------------------------------------
  // Robot Interface
  //---------------------------------------------------------------------------
  virtual void init( void );
  virtual void sense( void );
  virtual bool plan( void );
  virtual void act( void );

  //---------------------------------------------------------------------------
  bool plan_rrt( void );
  bool plan_simple( void );
 
  //---------------------------------------------------------------------------
  void control_position( ship_command_c& u );
  void control_rotation( ship_command_c& u );
  void control( ship_command_c& u );

  //---------------------------------------------------------------------------
  // Ship Interface
  //---------------------------------------------------------------------------

  ship_state_c state( void );

  ship_command_c compute_feedback( void );

  unsigned int state_step;
  void update_desired_state( void );
  ship_state_c interpolate_linear( const ship_state_c& q0, const ship_state_c& qf, const Real& deltat, const int& step ) const; 
  ship_state_c desired_state_0, desired_state_1;
  double desired_state_duration;

public:
  aabb_c aabb( const std::vector<double>& q );
  aabb_c aabb( void );
  bool intersects_any_obstacle( const aabb_c& mybb, aabb_c& obstacle );
  bool intersects_world_bounds( const aabb_c& mybb );

  void stop( void );
  //---------------------------------------------------------------------------
  // OMPL
  //---------------------------------------------------------------------------
public:

  void inv_dyn(const std::vector<double>& q, const std::vector<double>& qdot_des, std::vector<double>& u) const;

  // ode -- see below in eular_integrator_c for how called
  void operator()( const ompl::base::State* state, const ompl::control::Control* control, std::valarray<double>& dstate, ship_c* ship ) const;

  // update post integration
  void update( ompl::base::State* state, const std::valarray<double>& dstate ) const;

  //DirectedControlSamplePtr allocateDirectedControlSampler( const SpaceInformation* si );

/*
  //---------------------------------------------------------------------------
  // GSL
  //---------------------------------------------------------------------------
  gsl_rng* gslrng;


*/
  //---------------------------------------------------------------------------
  // Auditing 
  //---------------------------------------------------------------------------
  std::string audit_file_planned_commands;
  std::string audit_file_planned_states;
  std::string audit_file_actual_commands;
  std::string audit_file_actual_states;
  std::string audit_file_fb_commands;
  std::string audit_file_ff_commands;
  std::string audit_file_interp_states;
  std::string audit_file_fb_error;
  std::string audit_file_control_values;

  bool write_command_audit_header( const std::string& filename );
  bool write_state_audit_header( const std::string& filename );
  bool write_audit_datum( const std::string& filename, const std::pair<double,ship_command_c>& cmd );
  bool write_audit_datum( const std::string& filename, const std::pair<double,ship_state_c>& state );

  bool write_audit_data( const std::string& filename, const ship_state_list_t& list );
  bool write_audit_data( const std::string& filename, const ship_command_list_t& list );

  //---------------------------------------------------------------------------
  // Testing 
  //---------------------------------------------------------------------------
  void test_intersection_detection( void );

  //---------------------------------------------------------------------------
  // Planning
  //---------------------------------------------------------------------------
  ship_state_list_t states_actual;
  ship_state_list_t states_desired;

  ship_command_list_t commands_desired;

  ship_command_c command_current;
  double command_current_duration;

  bool is_state_valid(const ompl::control::SpaceInformation *si, const ompl::base::State *state);
};

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
class directed_control_sampler_c : public ompl::control::SimpleDirectedControlSampler {
public:
  const ompl::base::StateSpace *space_;
  ship_c* ship;

  //---------------------------------------------------------------------------
  directed_control_sampler_c( const ompl::control::SpaceInformation *si, const ompl::base::StateSpace* space, ship_c* _ship, unsigned int k = 1 ) :
    SimpleDirectedControlSampler(si, k) //, cs_(si->allocControlSampler()), numControlSamples_(k)
  {
    ship = _ship;
    space_ = space;
  }

  //---------------------------------------------------------------------------
  virtual ~directed_control_sampler_c( void ) {

  }

  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  virtual unsigned int getBestControl( ompl::control::Control *control, const ompl::base::State *source, ompl::base::State *dest, const ompl::control::Control *previous ) {

    const double DT = ship->PLANNER_STEP_SIZE;
    const unsigned NUM_STEPS = 1;

    // generate state vector(s)
    std::vector<double> q0( ship_state_c::size() );
    from_state( space_, source, q0 );
    std::vector<double> q1( ship_state_c::size() );
    from_state( space_, dest, q1 );

    //std::cout << "q0:" << q0 << std::endl;
    //std::cout << "q1:" << q1 << std::endl;

    // normalize the destination state quaternion
    double qnorm = 0.0;
    for( unsigned i = 3; i < 7; i++ )
      qnorm += q1[i] * q1[i];
    qnorm = std::sqrt( qnorm );
    for( unsigned i = 3; i < 7; i++ )
      q1[i] /= qnorm;

    // generate differential vector
    std::vector<double> dq( ship_state_c::size() );
    for( unsigned i = 0; i < ship_state_c::size(); i++ )
      dq[i] = q1[i] - q0[i];

    // scale differential vector
    for( unsigned i = 0; i < ship_state_c::size(); i++ )
      dq[i];// /= DT;

    // update dq to update position
    dq[7] += dq[0];///DT;
    dq[8] += dq[1];///DT;
    dq[9] += dq[2];///DT;

    // update dq to update velocity components
    Ravelin::Quatd edot(dq[3], dq[4], dq[5], dq[6]);
    Ravelin::Quatd e(q0[3], q0[4], q0[5], q0[6]);
    Ravelin::Vector3d omega = Ravelin::Quatd::to_omega(e, edot);
    dq[10] += omega[0];///DT;
    dq[11] += omega[1];///DT;
    dq[12] += omega[2];///DT;

    // call inverse dynamics
    std::vector<double> u;
    ship->inv_dyn( q0, dq, u );

    //std::cout << "u:" << u << std::endl;

    // copy u to control
    double *ctl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    for( unsigned i = 0; i < ship_command_c::size(); i++ )
      ctl[i] = u[i];

    // copy source to dest to start integrating
    to_state( space_, q0, dest );

    // integrate for the desired number of steps
    std::valarray<double> deltaq;
    for( unsigned k = 0; k < NUM_STEPS; k++ ) {
      // get the ODE
      (*ship)(dest, control, deltaq, ship );

      // first, update the velocity components
      for( unsigned i = 7; i < ship_state_c::size(); i++ ) {
        q0[i] += DT*deltaq[i];
      }

      // now update the position components (first)
      q0[0] += DT*q0[7];
      q0[1] += DT*q0[8];
      q0[2] += DT*q0[9];

      // convert the angular velocity to a quaternion time derivative 
      Ravelin::Quatd e(q0[3], q0[4], q0[5], q0[6]);
      Ravelin::Vector3d omega(q0[10], q0[11], q0[12]);
      Ravelin::Quatd edot = Ravelin::Quatd::deriv(e, omega);

      // update the orientation components using the angular velocity
      q0[3] += DT*edot[0];
      q0[4] += DT*edot[1];
      q0[5] += DT*edot[2];
      q0[6] += DT*edot[3];

      // normalize quaternion
      double qnorm = 0.0;
      for( unsigned i = 3; i < 7; i++ )
        qnorm += q0[i]*q0[i];
      qnorm = std::sqrt(qnorm);
      for( unsigned i = 3; i < 7; i++ )
        q0[i] /= qnorm;

      // update dest
      to_state(space_, q0, dest);

      //std::cout << "q0_dest[" << k << "]:" << q0 << std::endl;
    }

    // return 1
    return (unsigned) NUM_STEPS;
  }

};

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
ompl::control::DirectedControlSamplerPtr allocate_directed_control_sampler( const ompl::control::SpaceInformation* si ) {
  int k = 1;
  return ompl::control::DirectedControlSamplerPtr( new directed_control_sampler_c(si, ship->space_, ship,k) );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
template<typename F>
class euler_integrator_c {
private:
  const ompl::base::StateSpace *space;
  double time_step;
  F      ode;

  //---------------------------------------------------------------------------
public:
  euler_integrator_c( const ompl::base::StateSpace *_space, const double& _time_step ) : 
    space( _space ), 
    time_step( _time_step), 
    ode( _space )
  { }

  //---------------------------------------------------------------------------
  void propagate( const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result, ship_c* ship ) const {
    std::valarray<double> dstate;
    space->copyState( result, start );

    ship_state_c q( space, start );
    ship_command_c u( control );

    ode( result, control, dstate, ship );
    ode.update( result, time_step * dstate );
  }

  //---------------------------------------------------------------------------
  double get_time_step( void ) const {
    return time_step;
  }

  //---------------------------------------------------------------------------
  void set_time_step( const double& _time_step ) {
    time_step = _time_step;
  }
};

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
class state_propagator_c : public ompl::control::StatePropagator {
public:
  euler_integrator_c< ship_c > integrator;
  const ompl::base::StateSpace *space;
  ship_c* ship;

  //---------------------------------------------------------------------------
  state_propagator_c( const ompl::control::SpaceInformationPtr &si, ship_c* _ship) : 
    ship(_ship),
    ompl::control::StatePropagator(si),
    space( si->getStateSpace().get() ),
    integrator( si->getStateSpace().get(), 0.0 )
  { }

  //---------------------------------------------------------------------------
  virtual void propagate( const ompl::base::State* state, const ompl::control::Control* control, const double duration, ompl::base::State* result ) const {
    integrator.propagate( state, control, duration, result, ship );
  }

  //---------------------------------------------------------------------------
  void setIntegrationTimeStep( const double& time_step ) {
    integrator.set_time_step( time_step );
  }

  //---------------------------------------------------------------------------
  double getIntegrationTimeStep( void ) const {
    return integrator.get_time_step();
  }
};

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

class ship_goal_c : public ompl::base::GoalState {
public:
  const ompl::base::StateSpace *space;
  double SATISFACTION_THRESHOLD;

  //---------------------------------------------------------------------------
  ship_goal_c( const ompl::base::SpaceInformationPtr& si ) : 
    ompl::base::GoalState(si),
    space( si->getStateSpace().get() )
  {
    SATISFACTION_THRESHOLD = 1e-4;

  }

  //---------------------------------------------------------------------------
  virtual bool isSatisfied( const ompl::base::State* state ) const {
    ship_state_c q( space, state );
    if( distanceGoal( state ) < SATISFACTION_THRESHOLD ) return true;
    return false;
  }

  //---------------------------------------------------------------------------
  virtual bool isSatisfied( const ompl::base::State* state, double *distance ) const {
    ship_state_c q( space, state );

    *distance = distanceGoal( state );
    if( *distance < SATISFACTION_THRESHOLD ) return true;
    return false;
  }

  //---------------------------------------------------------------------------
  virtual double distanceGoal( const ompl::base::State* state ) const {
    std::vector<double> g( ship_state_c::size() );
    for( unsigned i=0; i< ship_state_c::size(); i++ )
      g[i] = value(i);
    ship_state_c q( space, state );
    std::vector<double> d( ship_state_c::size() );
    for( unsigned i = 0; i < d.size(); i++ ) 
      d[i] = q.value(i) - g[i];

    return sqrt( d[0] * d[0] + d[1] * d[1] + d[2] * d[2] );
  }

  //---------------------------------------------------------------------------
  double value( const unsigned int i ) const {
    assert( i < ship_state_c::size() );

    return *space->getValueAddressAtIndex(state_, i);
  }
};

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

class control_space_c : public ompl::control::RealVectorControlSpace {
public:
  control_space_c(const ompl::base::StateSpacePtr &stateSpace) : 
    ompl::control::RealVectorControlSpace(stateSpace, 6)
  { }
};

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

#endif // _GAZEBO_SHIP_H_

