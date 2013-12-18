#ifndef _GAZEBO_SHIP_H_
#define _GAZEBO_SHIP_H_

#include "constants.h"
//#include "integrator.h"
#include "control_space.h"
#include "goal.h"
#include "aabb.h"
#include "command.h"
#include "state.h"
#include "utilities.h"
#include "space.h"

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

class ship_c;
typedef boost::shared_ptr<ship_c> ship_p;

//-----------------------------------------------------------------------------
/// Class encapsulating the ship itself.  A Gazebo plugin
class ship_c : public gazebo::ModelPlugin {
public:

  //---------------------------------------------------------------------------
  // Open Parameters
  //---------------------------------------------------------------------------
  double FLEE_DISTANCE;
  double CAPTURE_DISTANCE;
  double PREY_MAX_FORCE;
  double PREY_MAX_TORQUE;
  double PREY_PREDATOR_FORCE_WEIGHT;
  double PREY_COMBINED_FORCE_WEIGHT;
  double PREY_BOUNDARY_FORCE_WEIGHT;

  // the time step used by the planner
  double PLANNER_STEP_SIZE;
  double PLANNER_MAX_PLANNING_TIME;
  double PLANNER_MAX_DERIVATIVE;
  double PLANNER_MAX_FORCE;
  double PLANNER_GOAL_BIAS;

  double FEEDBACK_GAIN_PROPORTIONAL_POSITION;
  double FEEDBACK_GAIN_DERIVATIVE_POSITION;
  double FEEDBACK_GAIN_PROPORTIONAL_ROTATION;
  double FEEDBACK_GAIN_DERIVATIVE_ROTATION;

  double DRAG;
  double REPULSIVE_FORCE_ALPHA;

  // Variables for the Random Walk planning
  double GAUSSIAN_MEAN;
  double GAUSSIAN_VARIANCE;
  double GAUSSIAN_STDDEV;

  //---------------------------------------------------------------------------
  // Members
  //---------------------------------------------------------------------------
  // the reference so that this ship is inserted into gazebo's callback system
  gazebo::event::ConnectionPtr updateConnection;
  // the gazebo reference to the world in which the ship is located
  gazebo::physics::WorldPtr world;
  // the gazebo reference to the ship's model 
  gazebo::physics::ModelPtr model;
  // the gazebo reference to the ship's link
  gazebo::physics::LinkPtr body;

  // information about the adversary/opponent 
  ship_p adversary;

  // the set of possible game types
  enum game_type_e {
    UNDEFINED,
    SOLO,
    PURSUIT
  };

  // the type of game that has started
  game_type_e GAME_TYPE;

  // the set of possible player types
  enum player_type_e {
    NONE,
    PREY,
    PREDATOR
  };

  // my player type
  player_type_e PLAYER_TYPE;
  // my adversary's player type
  player_type_e ADVERSARY_TYPE;

  // the inertial mass matrix of the ship
  Ravelin::SpatialRBInertiad inertial;

  // the current time of the simulation
  double time;

  // the size of the last time step the simulation took
  double dtime;

  // the time at which the simulation legitimately started
  double time_start;

  // the time of the last update
  double time_last;

  bool first_update;

  // whether the ship should stop interacting with the world
  bool stopped;

  // the ship's bounding box in the ship's frame of reference
  aabb_c ship_frame_bb;

  // the world's spatial boundary
  aabb_c spatial_bound;

  // the world's spatial information
  space_c space;

  // the list of all obstacles in the world
  aabb_list_t obstacles;

  // whether or not a capture event has occurred
  bool capture;

protected:
  // the type of planner this ship uses
  enum planner_type_e {
    WALK,
    RRT
  };

  // what planner is being used (defunct as of now)
  planner_type_e PLANNER;
public:

  // the current step between two states when interpolating
  unsigned int state_step;

  // the remaining time over which a state is valid when interpolating
  double desired_state_duration;

  // the current and next states when interpolating
  ship_state_c desired_state_0, desired_state_1;

  ship_state_c desired_state;

  // audit file names
  std::string audit_file_planned_commands;
  std::string audit_file_planned_states;
  std::string audit_file_actual_commands;
  std::string audit_file_actual_states;
  std::string audit_file_fb_commands;
  std::string audit_file_ff_commands;
  std::string audit_file_interp_states;
  std::string audit_file_fb_error;
  std::string audit_file_control_values;

  //---------------------------------------------------------------------------
  // Planning
  //---------------------------------------------------------------------------
  // the set of states the ship actually reaches
  ship_state_list_t states_actual;

  // the set of states the ship is expected to reach as generated by the planner
  ship_state_list_t states_desired;

  // the set of feedforward commands generated by the planner
  ship_command_list_t commands_desired;

  // the current command being executed by the ship
  ship_command_c command_current;

  // the time left the current command is to be executed
  double command_current_duration;

public:
  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
  ship_c( void );
  ship_c( ompl::base::StateSpace *_statespace );
  ship_c( ship_c* owner, const player_type_e& player_type );
    
  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~ship_c( void );

protected:
  //---------------------------------------------------------------------------
  // Gazebo ModelPlugin Interface
  //---------------------------------------------------------------------------
  // Gazebo callback.  Called when the simulation is starting up
  virtual void Load( gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf );

  // Gazebo callback.  Called whenever the simulation advances a timestep
  virtual void Update( );

  // Gazebo callback.  Called whenever the simulation is reset
  //virtual void Reset( );

  //---------------------------------------------------------------------------
  // Robot Interface
  //---------------------------------------------------------------------------
  // general initialization not directly related to gazebo (called by gazebo Init)
  virtual void init( void );

  // handles any sensing for a given time step
  virtual void sense( void );

  // handles any planning for a given time step
  virtual bool plan( void );

  // handles any acting behavior for a given time step
  virtual void act( void );

  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  bool read_model( gazebo::physics::ModelPtr _model );

  bool read_adversary( void );
  
  bool read_world( void );
  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  // plans using the ompl rrt planner
  bool plan_rrt( void );

  // plans a very simple trajectory for testing
  bool plan_simple( void );
 
  // controls the positional behavior of the ship given a command
  void control_position( ship_command_c& u );

  // controls the rotational behavior of the ship given a command
  void control_rotation( ship_command_c& u );

  // sends commands to all control components (encapsulates all sub controls)
  void control( ship_command_c& u );

  // send a command to stop the ship
  void stop( void );

  // query for the state of the ship as maintained by the simulator
  ship_state_c state( void );

  // computes the feedback command based on error in the current state
  ship_command_c compute_feedback( void );

  // updates the desired state for feedback control
  void update_desired_state( void );

  // compute the desired state based on the current command and current state
  void compute_desired_state( const ship_command_c& command, ship_state_c& result );

  // interpolates state given two states
  ship_state_c interpolate_linear( const ship_state_c& q0, const ship_state_c& qf, const double& deltat, const int& step ) const; 

  bool has_captured( const std::vector<double>& pred_state, const std::vector<double>& prey_state );

public:
  // computes commands (forces) if the ship is prey
  static void compute_prey_command( const std::vector<double>& pred_state, const std::vector<double>& prey_state, std::vector<double>& prey_u, const double& time, const double& dtime, space_c* space, ship_c* prey );

  // normalizes the quaternion components of the state
  static void renormalize_state_quat(std::vector<double>& q);

  // compute the bounding box for the ship given a state
  aabb_c aabb( const std::vector<double>& q );

  // query the bounding box for the ship in the current state
  aabb_c aabb( void );

  // query whether a bounding box intersects another bounding box
  bool intersects_any_obstacle( const aabb_c& mybb, aabb_c& obstacle );

  // query whether the ship intersects the world bounding box
  bool intersects_world_bounds( const aabb_c& mybb );

  // compute the inverse dynamics for the ship 
  static void inv_dyn( const std::vector<double>& q, const std::vector<double>& qdot_des, std::vector<double>& u, const ship_c* ship );

  static void ode( const std::vector<double>& q, const std::vector<double>& u, std::vector<double>& dq, const ship_c* ship );

  // compute any force(field) that the boundary contributes to repel collision
  static Ravelin::Vector3d boundary_force( ship_c* ship, space_c* space, const Ravelin::Vector3d& pos, const Ravelin::Vector3d& vel );

  // computes a repulsive force for a given distance
  static double repulsive_force( ship_c* ship, double dist );

  //---------------------------------------------------------------------------
  // Testing Method
  //---------------------------------------------------------------------------
  void test_intersection_detection( void );


  //---------------------------------------------------------------------------
  // OMPL
  //---------------------------------------------------------------------------
public:
  // the reference to the statespace the planner has generated
  ompl::base::StateSpace *statespace;

  // query whether or not the state generated by the planner is valid 
  bool is_state_valid(const ompl::control::SpaceInformation *si, const ompl::base::State *state);

  // allocates the directed control sampler for the planner
  ompl::control::DirectedControlSamplerPtr allocate_directed_control_sampler( const ompl::control::SpaceInformation* si );

  // allocates the predator/prey control sampler for the planner
  ompl::control::DirectedControlSamplerPtr allocate_pp_control_sampler( const ompl::control::SpaceInformation* si );

  // ode.  called by the eular_integrator_c in integrator.h
  void operator()( const ompl::base::State* state, const ompl::control::Control* control, std::vector<double>& dstate, const ship_c* ship ) const;

  // post integration update method
  void update( ompl::base::State* state, const std::vector<double>& dstate ) const;

};

//-----------------------------------------------------------------------------

#endif // _GAZEBO_SHIP_H_

