#ifndef _GAZEBO_CAR_H_
#define _GAZEBO_CAR_H_

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
//#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>

#include <ompl/control/ODESolver.h>

//-----------------------------------------------------------------------------

typedef double Real;

#define PI 3.14159265359

//-----------------------------------------------------------------------------

class car_state_c {
public:
  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
  car_state_c( void ) : 
    x(0.0), 
    y(0.0), 
    theta(0.0),
    time(0.0) 
  {}

  car_state_c( const car_state_c& _state ) : 
    x(_state.x), 
    y(_state.y),
    time(_state.time)
  {
    theta = _state.theta;
    while( theta > PI )
      theta -= 2.0 * PI;
    while( theta <= -PI )
      theta += 2.0 * PI;

  }

  car_state_c( const Real& _x, const Real& _y, const Real& _theta ) : 
    x(_x), 
    y(_y),
    time(0.0)
  {
    theta = _theta;
    while( theta > PI )
      theta -= 2.0 * PI;
    while( theta <= -PI )
      theta += 2.0 * PI;
  }

  car_state_c( const Real& _time, const Real& _x, const Real& _y, const Real& _theta ) : 
    x(_x), 
    y(_y),
    time(_time)
  {
    theta = _theta;
    while( theta > PI )
      theta -= 2.0 * PI;
    while( theta <= -PI )
      theta += 2.0 * PI;
  }

  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~car_state_c( void ) {}

  //---------------------------------------------------------------------------
  // Variables
  //---------------------------------------------------------------------------
  Real x;
  Real y;
  Real theta;
  Real time;

  //---------------------------------------------------------------------------
  // Output
  //---------------------------------------------------------------------------
  static std::string header( void ) {
    return "time, x, y, theta";
  }

  friend std::ostream& operator<<(std::ostream& ostr, const car_state_c& state);
};
car_state_c operator+( const car_state_c& a, const car_state_c& b ) {
  Real theta = a.theta + b.theta;
  while( theta > PI )
    theta -= 2.0 * PI;
  while( theta <= -PI )
    theta += 2.0 * PI;

  return car_state_c( a.time + b.time, a.x + b.x, a.y + b.y, theta );
}
car_state_c operator-( const car_state_c& a, const car_state_c& b ) {
  Real theta = a.theta - b.theta;
  while( theta > PI )
    theta -= 2.0 * PI;
  while( theta <= -PI )
    theta += 2.0 * PI;

  return car_state_c( a.time - b.time, a.x - b.x, a.y - b.y, theta );
}
car_state_c operator*( const Real& c, const car_state_c& a ) {
  Real theta = c * a.theta;
  while( theta > PI )
    theta -= 2.0 * PI;
  while( theta <= -PI )
    theta += 2.0 * PI;

  return car_state_c( c * a.time, c * a.x , c * a.y, theta );
}
car_state_c operator*( const car_state_c& a, const Real& c ) {
  Real theta = c * a.theta;
  while( theta > PI )
    theta -= 2.0 * PI;
  while( theta <= -PI )
    theta += 2.0 * PI;

  return car_state_c( c * a.time, c * a.x , c * a.y, theta );
}
car_state_c operator/( const car_state_c& a, const Real& c ) {
  const Real EPSILON = 1e-16;
  assert( fabs(c) > EPSILON );

  Real theta = a.theta / c;
  while( theta > PI )
    theta -= 2.0 * PI;
  while( theta <= -PI )
    theta += 2.0 * PI;

  return car_state_c( a.time / c, a.x / c, a.y / c, theta );
}
std::ostream& operator<<(std::ostream& ostr, const car_state_c& state) {
  return ostr << state.time << "," << state.x << "," << state.y << "," << state.theta;
}

//-----------------------------------------------------------------------------

class car_command_c {
public:
  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
  car_command_c( void ) : 
    duration(0.0),
    speed(0.0), 
    angle(0.0)
  {}

  car_command_c( const car_command_c& _command ) : 
    duration(_command.duration),
    speed(_command.speed), 
    angle(_command.angle)
  {}

  car_command_c( const Real& _speed, const Real& _angle ) : 
    duration(0.0),
    speed(_speed), 
    angle(_angle)
  {}

  car_command_c(const Real& _duration, const Real& _speed, const Real& _angle) :
    duration(_duration),
    speed(_speed), 
    angle(_angle)
  {}

  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~car_command_c( void ) {}

  //---------------------------------------------------------------------------
  // Variables
  //---------------------------------------------------------------------------
  Real duration; 
  Real speed;    // speed
  Real angle;    // steering angle

  //---------------------------------------------------------------------------
  // Output
  //---------------------------------------------------------------------------
  static std::string header( void ) {
    return "duration, speed, angle";
  }

  friend std::ostream& operator<<(std::ostream& ostr, const car_command_c& cmd);
};

car_command_c operator+( const car_command_c& a, const car_command_c& b ) {
  Real theta = a.angle + b.angle;
  while( theta > PI )
    theta -= 2.0 * PI;
  while( theta <= -PI )
    theta += 2.0 * PI;

  return car_command_c( a.speed + b.speed, theta );
}

std::ostream& operator<<(std::ostream& ostr, const car_command_c& cmd) {
  return ostr << cmd.duration << "," << cmd.speed << "," << cmd.angle;
}

//-----------------------------------------------------------------------------
class car_plan_c {
public:
  car_plan_c( void ) {}
  virtual ~car_plan_c( void ) {}

  std::vector<car_state_c> states;
  std::vector<car_command_c> commands;
 
};

//-----------------------------------------------------------------------------

class car_planner_c {
public:
  car_planner_c( void ) {}
  virtual ~car_planner_c( void ) {}

  enum car_planner_type_e {
    LISSAJOUS,
    RANDOM,
    OMPL
  };

};

//-----------------------------------------------------------------------------

class car_c : public gazebo::ModelPlugin {
private:

  gazebo::event::ConnectionPtr updateConnection;

  gazebo::physics::ModelPtr model;
  gazebo::physics::WorldPtr world;
  gazebo::physics::LinkPtr body;

  gazebo::physics::Joint_V joints;

  Real time, dtime, time_start, time_last;

  gazebo::physics::LinkPtr rear_driveshaft;
  gazebo::physics::LinkPtr steering_link;

  gazebo::physics::LinkPtr spindle_front_left;
  gazebo::physics::LinkPtr spindle_front_right;

  gazebo::physics::LinkPtr wheel_front_left;
  gazebo::physics::LinkPtr wheel_front_right;
  gazebo::physics::LinkPtr wheel_rear_left;
  gazebo::physics::LinkPtr wheel_rear_right;

  gazebo::physics::JointPtr steering_box;
  gazebo::physics::JointPtr rear_differential;

  gazebo::physics::JointPtr kingpin_front_left;
  gazebo::physics::JointPtr kingpin_front_right;

  gazebo::physics::JointPtr bearing_front_left;
  gazebo::physics::JointPtr bearing_front_right;
  gazebo::physics::JointPtr bearing_rear_left;
  gazebo::physics::JointPtr bearing_rear_right;

  Real MAX_SPEED;
  Real MAX_ACCELERATION;
  Real MAX_STEERING_VELOCITY;
  Real STEERING_CONTROL_KP;
  Real STEERING_CONTROL_KD;
  Real SPEED_CONTROL_KP;

  Real FEEDBACK_KS;
  Real FEEDBACK_KD;
  Real FEEDBACK_DU_SPEED;
  Real FEEDBACK_DU_ANGLE;
  Real FEEDBACK_MAX_SPEED;
  Real FEEDBACK_MAX_ANGLE;

public:
  Real WHEEL_BASE;
private:
  enum planner_type_e {
    LISSAJOUS,
    RRT
  };

  planner_type_e PLANNER;
  Real PLANNER_STEP_SIZE;

  enum steering_type_e {
    DIRECT,
    FOURBAR
  };

  steering_type_e STEERING;  

private:
  Real MAX_STEERING_ANGLE;
  Real STEERING_LEVER_BASE_ANGLE;
  Real BASE_LINK_LENGTH;
  Real STEERING_LEVER_LENGTH;
  Real TIEROD_LENGTH;
  Real SPINDLE_LEVER_LENGTH;
  Real SPINDLE_LEVER_BASE_ANGLE;

  Real BASE_LINK_LENGTH_SQ;
  Real STEERING_LEVER_LENGTH_SQ;
  Real TIEROD_LENGTH_SQ;
  Real SPINDLE_LEVER_LENGTH_SQ;

  //---------------------------------------------------------------------------
  // Testing Parameters
  //---------------------------------------------------------------------------
  Real LISSAJOUS_KS;
  Real LISSAJOUS_KD;
  Real LISSAJOUS_DU_SPEED;
  Real LISSAJOUS_DU_ANGLE;

  car_command_c ff_command;
  car_command_c fb_command;

  car_state_c desired_state_0;
  car_state_c desired_state_1;
  int state_step;
  Real desired_state_duration;

  car_state_c prev_state;

  void update_desired_state( void );

  bool stopped;

public:
  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
  car_c( void );
  car_c( const ompl::base::StateSpace *space );
    
  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~car_c( void );

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
  // Car Interface
  //---------------------------------------------------------------------------
  car_state_c state( void );
  car_state_c dstate( void );
  void state( const car_state_c& _state );
  gazebo::math::Vector3 orientation( void );
  Real steering_angle( void );
  Real speed( void );
  Real acceleration( void );

  void steer( const car_command_c& u, const Real& Kp, const Real& Kd );
  void push( const car_command_c& u, const Real& Kp );
  void stop( void );

  void steer_direct( const Real& _steering_angle );
  void steer_double_four_bar( const Real& _steering_angle );
  Real double_four_bar_kingpin_angle( const Real& _steering_angle );

  //void compensate_for_roll_pitch( void );

  car_command_c compute_feedback_command( void );

  //---------------------------------------------------------------------------
  // OMPL
  //---------------------------------------------------------------------------
  bool plan_via_ompl( void );
public:
  void operator()( const ompl::base::State *state, const ompl::control::Control *control, std::valarray<double> &dstate, const double& wheel_base ) const;
  void update(ompl::base::State *state, const std::valarray<double> &dstate) const;

protected:
  const ompl::base::StateSpace *space_;

  //---------------------------------------------------------------------------
  // ODE 
  //---------------------------------------------------------------------------
  car_state_c ode( const car_state_c& q, const car_command_c& u );
  car_command_c inverse_ode( const car_state_c& q, const car_state_c& dq );

  Real speed_optimization_function( const Real& u_s, const Real& x_dot, const Real& y_dot, const Real& costheta, const Real& sintheta );
  car_state_c interpolate_linear( const car_state_c& q0, const car_state_c& qf, const Real& deltat, const int& step );

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
  bool write_audit_datum(const std::string& filename, const car_command_c& cmd);
  bool write_audit_datum(const std::string& filename, const car_state_c& state);

  //---------------------------------------------------------------------------
  // Testing 
  //---------------------------------------------------------------------------
  void test_state_functions( void );
  void test_command_functions( void );

  //---------------------------------------------------------------------------
  // Planning
  //---------------------------------------------------------------------------
  std::vector<car_state_c> states;
  std::vector<car_state_c> desired_states;
  std::vector<car_command_c> commands;

  Real signed_angle( const Real& ux, const Real& uy, const Real& vx, const Real& vy ); 
  car_state_c lissajous( const Real& t, const Real& a, const Real& b, const Real& A, const Real& B, const Real& delta );
  bool plan_lissajous( void );

  //---------------------------------------------------------------------------

private:
};

//-----------------------------------------------------------------------------
// OMPL
//-----------------------------------------------------------------------------
// Following mostly borrowed wholesale from OMPL demo 
// RigidBodyPlanningWithIntegrationAndControls.cpp with refactoring to code style
//-----------------------------------------------------------------------------

bool is_state_valid(const ompl::control::SpaceInformation *si, const ompl::base::State *state) {
  //    ompl::base::ScopedState<ompl::base::SE2StateSpace>
  /// cast the abstract state type to the type we expect
  const ompl::base::SE2StateSpace::StateType *se2state = state->as<ompl::base::SE2StateSpace::StateType>();

  /// extract the first component of the state and cast it to what we expect
  const ompl::base::RealVectorStateSpace::StateType *pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0);

  /// extract the second component of the state and cast it to what we expect
  const ompl::base::SO2StateSpace::StateType *rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1);

  /// check validity of state defined by pos & rot
  // These constants are based on the size of the pen.
  if( fabs(pos->values[0]) >= 25.0 || fabs(pos->values[1]) >= 25.0 )
    return false;

  // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
  return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
}

//-----------------------------------------------------------------------------

class demo_control_space_c : public ompl::control::RealVectorControlSpace
{
public:

  demo_control_space_c(const ompl::base::StateSpacePtr &stateSpace) : 
    ompl::control::RealVectorControlSpace(stateSpace, 2)
  { }
};


//-----------------------------------------------------------------------------

/// Note: due to basing this on ompl demo, the typical integrator abstraction 
/// is broken because the ode has to at least know the wheel base which is a 
/// member of the instance and the steering angle is validated against the 
/// max_steering_angle for safety.
template<typename F>
class euler_integrator_c {
public:

  euler_integrator_c( const ompl::base::StateSpace *_space, const double& _time_step, const Real& _max_steering_angle, const Real& _wheel_base ) : 
    max_steering_angle( _max_steering_angle ),
    wheel_base( _wheel_base ),
    space( _space ), 
    time_step( _time_step), 
    ode( _space )
  { }

  void propagate( const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result ) const {
    double t = time_step;
    std::valarray<double> dstate;
    space->copyState( result, start );
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    if( fabs(u[1]) > max_steering_angle ) {
      // Note: state should already by copied from start to result so exiting means result is start state
      std::cout << "Control exceeds maximum steering angle [value:" << u[1] << "]\n";
      return;
    }
    while( t < duration + std::numeric_limits<double>::epsilon() ) {
      ode( result, control, dstate, wheel_base );
      ode.update( result, time_step * dstate );
      t += time_step;
    }
    if( t + std::numeric_limits<double>::epsilon() > duration ) {
      ode( result, control, dstate, wheel_base );
      ode.update( result, (t - duration) * dstate );
    }
  }

  double get_time_step( void ) const {
    return time_step;
  }

  void set_time_step( const double& _time_step ) {
    time_step = _time_step;
  }

protected:
  Real max_steering_angle;
  Real wheel_base;

private:
  const ompl::base::StateSpace *space;
  double time_step;
  F      ode;
};

//-----------------------------------------------------------------------------

class demo_state_propagator_c : public ompl::control::StatePropagator
{
public:

  demo_state_propagator_c( const ompl::control::SpaceInformationPtr &si, const Real& max_steering_angle, const double wheel_base ) : 
    ompl::control::StatePropagator(si),
    integrator( si->getStateSpace().get(), 0.0, max_steering_angle, wheel_base )
  { }

  virtual void propagate( const ompl::base::State *state, const ompl::control::Control* control, const double duration, ompl::base::State *result ) const {
    integrator.propagate( state, control, duration, result );
  }

  void setIntegrationTimeStep( const double& time_step ) {
    integrator.set_time_step( time_step );
  }

  double getIntegrationTimeStep( void ) const {
    return integrator.get_time_step();
  }

  euler_integrator_c< car_c > integrator;
};

//-----------------------------------------------------------------------------
/*
class rrt_c : public ompl::control::RRT {
public:
  rrt_c( const ompl::base::SpaceInformationPtr &si ) {
    ompl::control::RRT( si );
  }

  virtual ~rrt_c( void ) { }

};
*/

#endif // _GAZEBO_CAR_H_

