#ifndef _GAZEBO_CAR_H_
#define _GAZEBO_CAR_H_

//-----------------------------------------------------------------------------

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ostream>
#include <fstream>
#include <vector>
#include <string>

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
  Real MAX_STEERING_VELOCITY;
  Real STEERING_CONTROL_KP;
  Real STEERING_CONTROL_KD;
  Real SPEED_CONTROL_KP;

  Real WHEEL_BASE;


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

public:
  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
  car_c( void );

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

  void steer( const car_command_c& command );
  void push( const car_command_c& command );

  void steer_direct( const Real& _steering_angle );
  void steer_double_four_bar( const Real& _steering_angle );
  Real double_four_bar_kingpin_angle( const Real& _steering_angle );

  void compensate_for_roll_pitch( void );

  //---------------------------------------------------------------------------
  // ODE 
  //---------------------------------------------------------------------------
  car_state_c ode( const Real& theta, const car_command_c& command );
  car_command_c inverse_ode( const Real& theta, const car_state_c& dstate );

  //Real backtracking_line_search( const Real& u_s )
  Real speed_optimization_function( const Real& u_s, const Real& x_dot, const Real& y_dot, const Real& costheta, const Real& sintheta );

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
  std::vector<car_command_c> commands;

  Real signed_angle( const Real& ux, const Real& uy, const Real& vx, const Real& vy ); 
  car_state_c lissajous( const Real& t, const Real& a, const Real& b, const Real& A, const Real& B, const Real& delta );
  void plan_lissajous( void );

  //---------------------------------------------------------------------------

private:
};

//-----------------------------------------------------------------------------

#endif // _GAZEBO_CAR_H_

