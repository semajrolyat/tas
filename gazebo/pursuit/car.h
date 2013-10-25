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
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>

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
    theta(0.0) 
  {}

  car_state_c( const car_state_c& _state ) : 
    x(_state.x), 
    y(_state.y), 
    theta(_state.theta) 
  {}

  car_state_c( const Real& _x, const Real& _y, const Real& _theta ) : 
    x(_x), 
    y(_y), 
    theta(_theta) 
  {}

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

  //---------------------------------------------------------------------------
  // Output
  //---------------------------------------------------------------------------
  static std::string header( void ) {
    return "x, y, theta";
  }

  friend std::ostream& operator<<(std::ostream& ostr, const car_state_c& state);
};
car_state_c operator+( const car_state_c& a, const car_state_c& b ) {
  Real theta = a.theta + b.theta;
  while( theta > PI )
    theta -= 2 * PI;
  while( theta <= -PI )
    theta += 2 * PI;

  return car_state_c( a.x + b.x, a.y + b.y, theta );
}
car_state_c operator-( const car_state_c& a, const car_state_c& b ) {
  Real theta = a.theta - b.theta;
  while( theta > PI )
    theta -= 2 * PI;
  while( theta <= -PI )
    theta += 2 * PI;

  return car_state_c( a.x - b.x, a.y - b.y, theta );
}
car_state_c operator*( const Real& c, const car_state_c& a ) {
  Real theta = c * a.theta;
  while( theta > PI )
    theta -= 2 * PI;
  while( theta <= -PI )
    theta += 2 * PI;

  return car_state_c( c * a.x , c * a.y, theta );
}
car_state_c operator*( const car_state_c& a, const Real& c ) {
  Real theta = c * a.theta;
  while( theta > PI )
    theta -= 2 * PI;
  while( theta <= -PI )
    theta += 2 * PI;

  return car_state_c( c * a.x , c * a.y, theta );
}
std::ostream& operator<<(std::ostream& ostr, const car_state_c& state) {
  return ostr << state.x << "," << state.y << "," << state.theta;
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
  Real SPEED_CONTROL_KD;

  Real WHEEL_BASE;

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

  car_command_c active_command;
  car_state_c active_state;
  Real active_state_duration;

  car_state_c dstate;
  car_state_c prev_state;

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
  virtual void plan( void );
  virtual void act( void );

  //---------------------------------------------------------------------------
  // Car Interface
  //---------------------------------------------------------------------------
  car_state_c state( void );
  void state( const car_state_c& _state );
  gazebo::math::Vector3 orientation( void );
  Real steering_angle( void );
  Real speed( void );
  Real acceleration( void );

  void steer( const car_command_c& command );
  void push( const car_command_c& command );

  void steer_direct( const Real& _steering_angle );
  void steer_double_four_bar( const Real& _steering_angle );
  Real double_four_bar_kingpin_angle( const Real& _steering_angle );

  void compensate_for_roll_pitch( void );

  //---------------------------------------------------------------------------
  // OMPL
  //---------------------------------------------------------------------------
  void plan_via_ompl( void );
public:
  void operator()( const ompl::base::State *state, const ompl::control::Control *control, std::valarray<double> &dstate) const;
  void update(ompl::base::State *state, const std::valarray<double> &dstate) const;

protected:
  const ompl::base::StateSpace *space_;

  //---------------------------------------------------------------------------
  // ODE 
  //---------------------------------------------------------------------------
  car_state_c ode( const Real& theta, const car_command_c& command );
  car_command_c inverse_ode( const Real& theta, const car_state_c& dstate );

  //Real backtracking_line_search( const Real& u_s )
  Real lissajous_speed_optimization_function( const Real& u_s, const Real& x_dot, const Real& y_dot, const Real& costheta, const Real& sintheta );

  //---------------------------------------------------------------------------
  // Auditing 
  //---------------------------------------------------------------------------
  std::string audit_file_planned_commands;
  std::string audit_file_planned_states;
  std::string audit_file_actual_commands;
  std::string audit_file_actual_states;
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
  void plan_lissajous( void );

  //---------------------------------------------------------------------------

private:
};

//-----------------------------------------------------------------------------

/// Simple integrator: Euclidean method
template<typename F>
class EulerIntegrator {
public:

  EulerIntegrator(const ompl::base::StateSpace *space, double timeStep) : 
    space_(space), 
    timeStep_(timeStep), 
    ode_(space) 
  { }

  void propagate(const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result) const {
    double t = timeStep_;
    std::valarray<double> dstate;
    space_->copyState(result, start);
    while( t < duration + std::numeric_limits<double>::epsilon() ) {
      ode_( result, control, dstate );
      ode_.update( result, timeStep_ * dstate );
      t += timeStep_;
    }
    if( t + std::numeric_limits<double>::epsilon() > duration ) {
      ode_(result, control, dstate);
      ode_.update(result, (t - duration) * dstate);
    }
  }

  double getTimeStep(void) const {
    return timeStep_;
  }

  void setTimeStep(double timeStep) {
    timeStep_ = timeStep;
  }

private:
  const ompl::base::StateSpace *space_;
  double timeStep_;
  F      ode_;
};

//-----------------------------------------------------------------------------

bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state) {
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

/// @cond IGNORE
class DemoControlSpace : public ompl::control::RealVectorControlSpace
{
public:

  DemoControlSpace(const ompl::base::StateSpacePtr &stateSpace) : 
    ompl::control::RealVectorControlSpace(stateSpace, 2)
  { }
};

//-----------------------------------------------------------------------------

class DemoStatePropagator : public ompl::control::StatePropagator
{
public:

  DemoStatePropagator(const ompl::control::SpaceInformationPtr &si) : 
    ompl::control::StatePropagator(si), integrator_(si->getStateSpace().get(), 0.0)
  { }

  virtual void propagate(const ompl::base::State *state, const ompl::control::Control* control, const double duration, ompl::base::State *result) const {
    integrator_.propagate(state, control, duration, result);
  }

  void setIntegrationTimeStep(double timeStep) {
    integrator_.setTimeStep(timeStep);
  }

  double getIntegrationTimeStep(void) const {
    return integrator_.getTimeStep();
  }

  EulerIntegrator< car_c > integrator_;
};

//-----------------------------------------------------------------------------

#endif // _GAZEBO_CAR_H_

