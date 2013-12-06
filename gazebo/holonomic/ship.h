#ifndef _GAZEBO_SHIP_H_
#define _GAZEBO_SHIP_H_

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
std::ostream& operator<<(std::ostream& ostr, const std::vector<double>& v) {
  for( unsigned i = 0; i < v.size(); i++ ) {
    if( i > 0 )
      ostr << ",";
    ostr << v[i];
  }
  return ostr;
}


//-----------------------------------------------------------------------------
class ship_c;
ship_c* ship;

//-----------------------------------------------------------------------------

class ship_state_c {
  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
public:
  //---------------------------------------------------------------------------
  ship_state_c( void ) {

    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = 0.0;
    _values[6] = 1.0; // Normalize w coordinate
  }

  //---------------------------------------------------------------------------
  ship_state_c( const ship_state_c& state ) { 

    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = state._values[i];
  }

  //---------------------------------------------------------------------------
  ship_state_c( std::vector<double>& x ) { 
    assert( x.size() == size() );

    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = x[i];
  }

  //---------------------------------------------------------------------------
  ship_state_c( std::valarray<double>& x ) { 
    assert( x.size() == size() );

    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = x[i];
  }

  //---------------------------------------------------------------------------
  ship_state_c( gazebo::math::Vector3 x, gazebo::math::Quaternion theta, gazebo::math::Vector3 dx, gazebo::math::Vector3 dtheta ) { 
    _values.resize( size() );
    position( x );
    rotation( theta );
    dposition( dx );
    drotation( dtheta );
  }

  //---------------------------------------------------------------------------
  ship_state_c( const ompl::base::StateSpace* space, const ompl::base::State* x ) { 
    //assert( x.size() == size() );

    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = *space->getValueAddressAtIndex(x, i);
  }

  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~ship_state_c( void ) {}

  //---------------------------------------------------------------------------
  //
  //---------------------------------------------------------------------------
  void position( const gazebo::math::Vector3& v ) {
    _values[0] = v.x;
    _values[1] = v.y;
    _values[2] = v.z;
  }

  //---------------------------------------------------------------------------
  gazebo::math::Vector3 position( void ) const {
    return gazebo::math::Vector3( _values[0], _values[1], _values[2] );
  }

  //---------------------------------------------------------------------------
  void rotation( const gazebo::math::Quaternion& q ) {
    _values[3] = q.x;
    _values[4] = q.y;
    _values[5] = q.z;
    _values[6] = q.w;
  }

  //---------------------------------------------------------------------------
  gazebo::math::Quaternion rotation( void ) const {
    return gazebo::math::Quaternion( _values[6], _values[3], _values[4], _values[5] );
  }

  //---------------------------------------------------------------------------
  void dposition( const gazebo::math::Vector3& v ) {
    _values[7] = v.x;
    _values[8] = v.y;
    _values[9] = v.z;
  }

  //---------------------------------------------------------------------------
  gazebo::math::Vector3 dposition( void ) const {
    return gazebo::math::Vector3( _values[7], _values[8], _values[9] );
  }

  //---------------------------------------------------------------------------
  void drotation( const gazebo::math::Vector3& v ) {
    _values[10] = v.x;
    _values[11] = v.y;
    _values[12] = v.z;
  }

  //---------------------------------------------------------------------------
  gazebo::math::Vector3 drotation( void ) const {
    return gazebo::math::Vector3( _values[10], _values[11], _values[12] );
  }

  //---------------------------------------------------------------------------
  double& operator()( const unsigned& i ) {
    assert( i < size() );
    return _values[i];
  }

  //---------------------------------------------------------------------------
  double value( const unsigned& i ) const {
    assert( i < size() );
    return _values[i];
  }

  //---------------------------------------------------------------------------
  static unsigned size( void ) {
    return 13;
  }

  //---------------------------------------------------------------------------
  // Member Variables
  //---------------------------------------------------------------------------
private:
  std::vector<double> _values;
 
  //---------------------------------------------------------------------------
  // Output
  //---------------------------------------------------------------------------
public:
  static std::string header( void ) {
    return "t, pos[x, y, z], rot[x, y, z, w], dpos[x, y, z], drot[x, y, z]";
  }

  friend std::ostream& operator<<(std::ostream& ostr, const ship_state_c& state);
  friend std::ostream& operator<<(std::ostream& ostr, const std::pair<double,ship_state_c>& state);
};

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const ship_state_c& state) {
  return ostr << state.value(0) << "," << state.value(1) << "," << state.value(2) << "," 
              << state.value(3) << "," << state.value(4) << "," << state.value(5) << "," << state.value(6) << "," 
              << state.value(7) << "," << state.value(8) << "," << state.value(9) << "," 
              << state.value(10) << "," << state.value(11) << "," << state.value(12);
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const std::pair<double,ship_state_c>& state) {
  return ostr << state.first << ","
              << state.second.value(0) << "," << state.second.value(1) << "," << state.second.value(2) << "," 
              << state.second.value(3) << "," << state.second.value(4) << "," << state.second.value(5) << "," << state.second.value(6) << "," 
              << state.second.value(7) << "," << state.second.value(8) << "," << state.second.value(9) << "," 
              << state.second.value(10) << "," << state.second.value(11) << "," << state.second.value(12);
}

//-----------------------------------------------------------------------------
// Note for the below arithmetic operations, need to normalize the quaternion after the computation
ship_state_c operator+( ship_state_c& a, ship_state_c& b ) {

  ship_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) += b(i);
  return state;
}

//-----------------------------------------------------------------------------
ship_state_c operator-( ship_state_c& a, ship_state_c& b ) {

  ship_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) -= b(i);
  return state;
}

//-----------------------------------------------------------------------------
ship_state_c operator*( const double& c, ship_state_c& a ) {
  ship_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) *= c;
  return state;
}

//-----------------------------------------------------------------------------
ship_state_c operator*( ship_state_c& a, const double& c ) {
  ship_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) *= c;
  return state;
}

//-----------------------------------------------------------------------------
ship_state_c operator/( ship_state_c& a, const double& c ) {
  const Real EPSILON = 1e-16;
  assert( fabs(c) > EPSILON );

  ship_state_c state( a );
  for( unsigned i = 0; i < state.size(); i++ )
    state(i) /= c;
  return state;
}

//-----------------------------------------------------------------------------

class ship_command_c {

  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
public:
  //---------------------------------------------------------------------------
  ship_command_c( void ) {
    _values.resize( size() );

    for( unsigned i = 0; i < size(); i++ )
      _values[i] = 0.0;
  }

  //---------------------------------------------------------------------------
  ship_command_c( const ship_command_c& command ) {
    _values.resize( size() );

    for( unsigned i = 0; i < size(); i++ )
      _values[i] = command._values[i];
  }

  //---------------------------------------------------------------------------
  ship_command_c( std::vector<double>& x ) { 
    assert( x.size() == size() );

    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = x[i];
  }

  //---------------------------------------------------------------------------
  ship_command_c( std::valarray<double>& x ) { 
    assert( x.size() == size() );

    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = x[i];
  }

  //---------------------------------------------------------------------------
  ship_command_c( const double* x ) { 

    // Note there are pointer safety issues here
    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = x[i];
  }

  //---------------------------------------------------------------------------
  ship_command_c( gazebo::math::Vector3 F, gazebo::math::Vector3 tau ) { 
    _values.resize( size() );
    force( F );
    torque( tau );
  }

  //---------------------------------------------------------------------------
  ship_command_c( const ompl::control::Control* u ) { 
    //assert( x.size() == size() );

    _values.resize( size() );
    for( unsigned i = 0; i < size(); i++ )
      _values[i] = u->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i];
  }

  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~ship_command_c( void ) {}

  //---------------------------------------------------------------------------
  //
  //---------------------------------------------------------------------------
  void force( const gazebo::math::Vector3& v ) {
    _values[0] = v.x;
    _values[1] = v.y;
    _values[2] = v.z;
  }

  void force( const Ravelin::Vector3d& v ) {
    _values[0] = v[0];
    _values[1] = v[1];
    _values[2] = v[2];
  }

  //---------------------------------------------------------------------------
  gazebo::math::Vector3 force( void ) const {
    return gazebo::math::Vector3( _values[0], _values[1], _values[2] );
  }

  //---------------------------------------------------------------------------
  void torque( const gazebo::math::Vector3& v ) {
    _values[3] = v.x;
    _values[4] = v.y;
    _values[5] = v.z;
  }

  void torque( const Ravelin::Vector3d& v ) {
    _values[3] = v[0];
    _values[4] = v[1];
    _values[5] = v[2];
  }


  //---------------------------------------------------------------------------
  gazebo::math::Vector3 torque( void ) const {
    return gazebo::math::Vector3( _values[3], _values[4], _values[5] );
  }

  //---------------------------------------------------------------------------
  double& operator()( const unsigned& i ) {
    assert( i < size() );
    return _values[i];
  }

  //---------------------------------------------------------------------------
  double value( const unsigned& i ) const {
    assert( i < size() );
    return _values[i];
  }

  //---------------------------------------------------------------------------
  static unsigned size( void ) {
    return 6;
  }

  //---------------------------------------------------------------------------
  // Member Variables
  //---------------------------------------------------------------------------
private:
  std::vector<double> _values;
  double duration; 

  //---------------------------------------------------------------------------
  // Output
  //---------------------------------------------------------------------------
public: 
  static std::string header( void ) {
    return "t,force[x, y, z], torque[x, y, z]";
  }

  friend std::ostream& operator<<(std::ostream& ostr, const std::pair<double,ship_command_c>& cmd);
  friend std::ostream& operator<<(std::ostream& ostr, const ship_command_c& cmd);
};

//-----------------------------------------------------------------------------
ship_command_c operator+( ship_command_c& a, ship_command_c& b ) {
  ship_command_c command( a );
  for( unsigned i = 0; i < command.size(); i++ )
    command(i) += b(i);
  return command;
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const ship_command_c& cmd) {
  return ostr << cmd.value(0) << "," << cmd.value(1) << "," << cmd.value(2) << "," 
              << cmd.value(3) << "," << cmd.value(4) << "," << cmd.value(5);
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const std::pair<double,ship_command_c>& cmd) {
  return ostr << cmd.first << ","
              << cmd.second.value(0) << "," << cmd.second.value(1) << "," << cmd.second.value(2) << "," 
              << cmd.second.value(3) << "," << cmd.second.value(4) << "," << cmd.second.value(5);
}

//-----------------------------------------------------------------------------

typedef std::vector< std::pair<double,ship_state_c> > ship_state_list_t;
typedef std::vector< std::pair<double,ship_command_c> > ship_command_list_t;

/*
//-----------------------------------------------------------------------------
class ship_plan_c {
public:
  ship_plan_c( void ) {}
  virtual ~ship_plan_c( void ) {}

  std::vector<ship_state_c> states;
  std::vector<ship_command_c> commands;
 
};

//-----------------------------------------------------------------------------

class ship_planner_c {
public:
  ship_planner_c( void ) {}
  virtual ~ship_planner_c( void ) {}

  enum ship_planner_type_e {
    LISSAJOUS,
    RANDOM,
    OMPL
  };

};
*/
//-----------------------------------------------------------------------------

class ship_c : public gazebo::ModelPlugin {
protected:

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
  static double unit_rand( );

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
  ship_state_c ode( ship_state_c& q, ship_command_c& u ) const;
  ship_command_c inverse_ode( ship_state_c& q, ship_state_c& dq ) const;

  ship_state_c state( void );

  ship_command_c compute_feedback( void );

  unsigned int state_step;
  void update_desired_state( void );
  ship_state_c interpolate_linear( const ship_state_c& q0, const ship_state_c& qf, const Real& deltat, const int& step ) const; 
  ship_state_c desired_state_0, desired_state_1;
  double desired_state_duration;
/*
  ship_state_c dstate( void );

  void state( const ship_state_c& state_ );

  void steer( const car_command_c& u, const Real& Kp, const Real& Kd );
  void push( const car_command_c& u, const Real& Kp );
*/
  void stop( void );
  //---------------------------------------------------------------------------
  // OMPL
  //---------------------------------------------------------------------------
public:

  void inv_dyn(const std::vector<double>& q, const std::vector<double>& qdot_des, std::vector<double>& u) const;
  void operator()( const ompl::base::State* state, const ompl::control::Control* control, std::valarray<double>& dstate, ship_c* ship ) const;
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
/*
  //---------------------------------------------------------------------------
  // Testing 
  //---------------------------------------------------------------------------
  void test_state_functions( void );
  void test_command_functions( void );
*/
  //---------------------------------------------------------------------------
  // Planning
  //---------------------------------------------------------------------------
  ship_state_list_t states_actual;
  ship_state_list_t states_desired;

  ship_command_list_t commands_desired;

  ship_command_c command_current;
  double command_current_duration;

};

//-----------------------------------------------------------------------------
// OMPL
//-----------------------------------------------------------------------------
// Following mostly borrowed wholesale from OMPL demo 
// RigidBodyPlanningWithIntegrationAndControls.cpp with refactoring to code style
//-----------------------------------------------------------------------------
void to_state( const ompl::base::StateSpace* space, const std::vector<double>& values, ompl::base::State* state );
void from_state( const ompl::base::StateSpace* space, const ompl::base::State* state, std::vector<double>& values );

ompl::control::DirectedControlSamplerPtr allocateDirectedControlSampler( const ompl::control::SpaceInformation* si );

class demo_control_space_c : public ompl::control::RealVectorControlSpace
{
public:

  demo_control_space_c(const ompl::base::StateSpacePtr &stateSpace) : 
    ompl::control::RealVectorControlSpace(stateSpace, 6)
  { }
};

//-----------------------------------------------------------------------------

bool is_state_valid(const ompl::control::SpaceInformation *si, const ompl::base::State *state);

//-----------------------------------------------------------------------------

template<typename F>
class euler_integrator_c {
public:

  euler_integrator_c( const ompl::base::StateSpace *_space, const double& _time_step ) : 
    space( _space ), 
    time_step( _time_step), 
    ode( _space )
  { }
/*
  void propagate( const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result ) const {
    double t = time_step;
    std::valarray<double> dstate;
    space->copyState( result, start );
    while( t < duration + std::numeric_limits<double>::epsilon() ) {
      ode( result, control, dstate );
      ode.update( result, time_step * dstate );
      t += time_step;
    }
    if( t + std::numeric_limits<double>::epsilon() > duration ) {
      ode( result, control, dstate );
      ode.update( result, (t - duration) * dstate );
    }
  }
*/
//-----------------------------------------------------------------------------
  void propagate( const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result, ship_c* ship ) const {
    std::valarray<double> dstate;
    space->copyState( result, start );  // safety copy => if we need to get out early then initial state retained
/*
    const double *pu = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    std::vector<double> vq( ship_state_c::size() );
    std::vector<double> vdq( ship_state_c::size() );
    from_state( space, result, vq );
    ship_state_c q( vq );
    ship_command_c u( pu );
*/
    ship_state_c q( space, start );
    ship_command_c u( control );
    
    //std::cout << "prop_in: q[" << q << "], u[" << u << "]\n";

    ode( result, control, dstate, ship );
    ode.update( result, time_step * dstate );
    //std::cout << time_step << std::endl;

    //ode.update( result, dstate );
/*
    from_state( space, result, vq );
    q = ship_state_c( vq );
    from_state( space, result, vdq );
    ship_state_c dq( vdq );
  */  
    //std::cout << "prop_out: q[" << q << "], dq[" << dq << "]\n";
  }

  double get_time_step( void ) const {
    return time_step;
  }

  void set_time_step( const double& _time_step ) {
    time_step = _time_step;
  }

private:
  const ompl::base::StateSpace *space;
  double time_step;
  F      ode;
};

//-----------------------------------------------------------------------------

class demo_state_propagator_c : public ompl::control::StatePropagator
{
public:

  demo_state_propagator_c( const ompl::control::SpaceInformationPtr &si, ship_c* _ship) : 
    ship(_ship),
    ompl::control::StatePropagator(si),
    space( si->getStateSpace().get() ),
    integrator( si->getStateSpace().get(), 0.0 )
  { }

  virtual void propagate( const ompl::base::State* state, const ompl::control::Control* control, const double duration, ompl::base::State* result ) const {
    integrator.propagate( state, control, duration, result, ship );
  }
/*
  bool steer( const ompl::base::State* from, const ompl::base::State* to, const ompl::control::Control* u, double& duration ) const {
    //std::valarray<double> dstate;
    ship_state_c q0( space, from );
    ship_state_c q1( space, to );
    ship_state_c dq = q1 - q0;
    double dt = integrator.get_time_step();

    std::cout << "steer(...) ran\n";
    u->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0] = dq(7);
    u->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1] = dq(8);
    u->as<ompl::control::RealVectorControlSpace::ControlType>()->values[2] = dq(9);
    u->as<ompl::control::RealVectorControlSpace::ControlType>()->values[3] = dq(10);
    u->as<ompl::control::RealVectorControlSpace::ControlType>()->values[4] = dq(11);
    u->as<ompl::control::RealVectorControlSpace::ControlType>()->values[5] = dq(12);

    duration = dt;

    return true;
  }
*/
  void setIntegrationTimeStep( const double& time_step ) {
    integrator.set_time_step( time_step );
  }

  double getIntegrationTimeStep( void ) const {
    return integrator.get_time_step();
  }

  euler_integrator_c< ship_c > integrator;
  const ompl::base::StateSpace *space;
  ship_c* ship;

};

//-----------------------------------------------------------------------------
class rrt_c : public ompl::control::RRT {
public:
  
  rrt_c( const ompl::control::SpaceInformationPtr& si ) : ompl::control::RRT( si ) { }

  virtual ~rrt_c( void ) { }

};

//-----------------------------------------------------------------------------

class ship_goal_c : public ompl::base::GoalState {
public:
  double SATISFACTION_THRESHOLD;

  ship_goal_c( const ompl::base::SpaceInformationPtr& si ) : 
    ompl::base::GoalState(si),
    space( si->getStateSpace().get() )
  {
    SATISFACTION_THRESHOLD = 1e-4;

  }

  virtual bool isSatisfied( const ompl::base::State* state ) const {
    ship_state_c q( space, state );
    if( distanceGoal( state ) < SATISFACTION_THRESHOLD ) return true;
    return false;
  }

  virtual bool isSatisfied( const ompl::base::State* state, double *distance ) const {
    ship_state_c q( space, state );

    *distance = distanceGoal( state );
    if( *distance < SATISFACTION_THRESHOLD ) return true;
    return false;
  }

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

  double value( const unsigned int i ) const {
    assert( i < ship_state_c::size() );

    return *space->getValueAddressAtIndex(state_, i);
  }

  const ompl::base::StateSpace *space;
};

//-----------------------------------------------------------------------------

class ship_directed_control_sampler_c : public ompl::control::SimpleDirectedControlSampler {
public:
  ship_directed_control_sampler_c( const ompl::control::SpaceInformation *si, const ompl::base::StateSpace* space, ship_c* _ship, unsigned int k = 1 ) :
    SimpleDirectedControlSampler(si, k) //, cs_(si->allocControlSampler()), numControlSamples_(k)
  {
    ship = _ship;
    space_ = space;
  }

  virtual ~ship_directed_control_sampler_c( void ) {

  }

  const ompl::base::StateSpace *space_;
  ship_c* ship;


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


/*
    // Sample the first control
    if (previous)
      cs_->sampleNext(control, previous, source);
    else
      cs_->sample(control, source);

    const unsigned int minDuration = si_->getMinControlDuration();
    const unsigned int maxDuration = si_->getMaxControlDuration();

    unsigned int steps = cs_->sampleStepCount(minDuration, maxDuration);
    // Propagate the first control, and find how far it is from the target state
    ompl::base::State *bestState   = si_->allocState();
    steps = si_->propagateWhileValid(source, control, steps, bestState);

    if (numControlSamples_ > 1) {
      ompl::control::Control     *tempControl = si_->allocControl();
      ompl::base::State *tempState   = si_->allocState();
      double bestDistance      = si_->distance(tempState, dest);

      // Sample k-1 more controls, and save the control that gets closest to target
      for (unsigned int i = 1; i < numControlSamples_; ++i) {
        unsigned int sampleSteps = cs_->sampleStepCount(minDuration, maxDuration);
        if (previous)
          cs_->sampleNext(tempControl, previous, source);
        else
          cs_->sample(tempControl, source);

        sampleSteps = si_->propagateWhileValid(source, tempControl, sampleSteps, tempState);
        double tempDistance = si_->distance(tempState, dest);
        if (tempDistance < bestDistance) {
          si_->copyState(bestState, tempState);
          si_->copyControl(control, tempControl);
          bestDistance = tempDistance;
          steps = sampleSteps;
        }
      }

      si_->freeState(tempState);
      si_->freeControl(tempControl);
    }

    si_->copyState(dest, bestState);
    si_->freeState(bestState);

    return steps;
*/
  }

};

#endif // _GAZEBO_SHIP_H_

