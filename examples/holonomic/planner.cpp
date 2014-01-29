/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

planner.cpp
-----------------------------------------------------------------------------*/

#include "planner.h"

#include <Ravelin/Quatd.h>
#include <Ravelin/Vector3d.h>

//-----------------------------------------------------------------------------

//using namespace Moby;
//using boost::shared_ptr;
//using boost::dynamic_pointer_cast;

//-----------------------------------------------------------------------------


//#if defined(MOBY_DYNAMICS)
//-----------------------------------------------------------------------------
// Standalone Controller
//----------------------------------------------------------------------------

/*
// error logging facilities
log_c error_log;
char strbuffer[256];

//-----------------------------------------------------------------------------

// shared buffer for actuator messages
sharedbuffer_c amsgbuffer;

//-----------------------------------------------------------------------------

cpuinfo_c cpuinfo;
unsigned long long cpu_speed_hz;

//-----------------------------------------------------------------------------
*/

//-----------------------------------------------------------------------------
planner_c::planner_c( void ) {
  
}

//-----------------------------------------------------------------------------
planner_c::planner_c( const planner_type_e& type, ode_f _ode, best_control_f _bc, prey_command_f _pc, aabb_c _spatial_bound, ship_p _self, ship_p _adversary, const double& step_size, const double& max_plan_time, const double& max_derivative, const double& max_force, const double& goal_bias ) {
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

//-----------------------------------------------------------------------------
planner_c::planner_c( char* argv[] ) {
//  const unsigned X = 0, Y = 1, Z = 2;

  self = ship_p( new ship_c( argv[2] ) );
  adversary = ship_p( new ship_c( argv[3] ) );
  space = space_p( new space_c( argv[4] ) );
/*
  std::string name = argv[0];

  DT = atof(argv[1]);

  int id = atoi(argv[2]);
  // Note: this might be able to be pulled from name instead
  if( id == 1 ) {
    ship->PLAYER_TYPE = ship_c::PREDATOR;
    ship->ADVERSARY_TYPE = ship_c::PREY;
  } else {
    ship->PLAYER_TYPE = ship_c::PREY;
    ship->ADVERSARY_TYPE = ship_c::PREDATOR;
  }

  // setup mass
  ship->inertial.m = atof(argv[3]);

  // setup moment of inertia
  // Row X
  ship->inertial.J(X,X) = atof(argv[4]);  
  ship->inertial.J(X,Y) = atof(argv[5]);  
  ship->inertial.J(X,Z) = atof(argv[6]);

  // Row Y
  ship->inertial.J(Y,X) = atof(argv[7]);   
  ship->inertial.J(Y,Y) = atof(argv[8]); 
  ship->inertial.J(Y,Z) = atof(argv[9]);

  // Row Z
  ship->inertial.J(Z,X) = atof(argv[10]);   
  ship->inertial.J(Z,Y) = atof(argv[11]);  
  ship->inertial.J(Z,Z) = atof(argv[12]);

  FEEDBACK_GAIN_PROPORTIONAL_POSITION = atof(argv[13]);
  FEEDBACK_GAIN_PROPORTIONAL_POSITION = atof(argv[14]);
  FEEDBACK_GAIN_PROPORTIONAL_POSITION = atof(argv[15]);
  FEEDBACK_GAIN_PROPORTIONAL_POSITION = atof(argv[16]);
 */
/*
  PLANNER_STEP_SIZE;
  PLANNER_MAX_PLANNING_TIME;
  PLANNER_MAX_DERIVATIVE;
  PLANNER_MAX_FORCE;
  PLANNER_GOAL_BIAS;
*/
}

//-----------------------------------------------------------------------------
planner_c::~planner_c( void ) {
  
}

//-----------------------------------------------------------------------------
void planner_c::execute( void ) {
  // 
}

//-----------------------------------------------------------------------------
error_e planner_c::plan( const act_msg_c& input, act_msg_c& output ) {

  pp_state_c q = input.state();
  ship_command_c u;
/*
  if( !self->plan( q, u ) )
    return ERROR_FAILED; 
*/
  double time = input.header.time.seconds;
  bool result = plan_for_predator( time, q, u );

  if( !result )
    return ERROR_FAILED; 

  output.state( q );
  output.command( u );

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Send a message to the system to request state for a given time
error_e planner_c::request( const simtime_t& simtime, act_msg_c& state ) {

  char buf = 0;
  act_msg_c req;
  notification_c notification;

  // build the request message
  req.header.type = MSG_REQUEST;
  req.header.time = simtime;
  req.header.requestor = REQUESTOR_PLANNER;

  if( msg_buffer.write( req ) != BUFFER_ERR_NONE ) {
    sprintf( err_buffer, "(planner.cpp) request(simtime) failed calling sharedbuffer_c.write(req)\n" );
    error_log.write( err_buffer );
    return ERROR_FAILED;
  }

  notification.type = NOTIFICATION_REQUEST;
  notification.ts = generate_timestamp( );

  // send a notification to the coordinator that a message has been published
  if( write( FD_PLANNER_TO_COORDINATOR_WRITE_CHANNEL, &notification, sizeof(notification_c) ) == -1 ) {
    std::string err_msg = "(planner.cpp) request(simtime) failed making system call write(...)";
    error_log.write( error_string_bad_write( err_msg , errno ) );
    return ERROR_FAILED;
  }

  // wait for the reply meaning the state request fulfilled and written to the buffer
  // Note: controller blocks here on read
  if( read( FD_COORDINATOR_TO_PLANNER_READ_CHANNEL, &notification, sizeof(notification_c) ) == -1 ) {
    std::string err_msg = "(planner.cpp) request(simtime) failed making system call read(...)";
    error_log.write( error_string_bad_read( err_msg , errno ) );
    return ERROR_FAILED;
  }

  // At this point a message was sent from the coordinator.  Attempt to read from buffer
  // read the state out of the buffer
  // Note: will block waiting to acquire mutex
  if( msg_buffer.read( state ) != BUFFER_ERR_NONE ) {
    sprintf( err_buffer, "(planner.cpp) request(simtime) failed calling sharedbuffer_c.read(state)\n" );
    error_log.write( err_buffer );
    return ERROR_FAILED;
  }

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
error_e planner_c::publish( const act_msg_c& command ) {

  notification_c notification;

  // send the command message to the actuator
  // Note: will block waiting to acquire mutex
  if( msg_buffer.write( command ) != BUFFER_ERR_NONE) {
    sprintf( err_buffer, "(planner.cpp) publish(command) failed calling sharedbuffer_c.write(command)\n" );
    error_log.write( err_buffer );
    return ERROR_FAILED;
  }

  notification.type = NOTIFICATION_REQUEST;
  notification.ts = generate_timestamp( );

  // send a notification to the coordinator
  if( write( FD_PLANNER_TO_COORDINATOR_WRITE_CHANNEL, &notification, sizeof(notification_c) ) == -1 ) {
    std::string err_msg = "(planner.cpp) publish(command) failed making system call write(...)";
    error_log.write( error_string_bad_write( err_msg , errno ) );
    return ERROR_FAILED;
  }

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
// Fulfills the steps in a cycle
error_e planner_c::activate( const simtime_t& simtime ) {

  act_msg_c state, command;

  // query the dyanmics for the initial state
  if( request( simtime, state ) != ERROR_NONE ) {
    sprintf( err_buffer, "(planner.cpp) activate() failed calling request(simtime(%f),state)\n", simtime.seconds );
    error_log.write( err_buffer );
    return ERROR_FAILED;
  }

  // compute the initial command message
  if( plan( state, command ) != ERROR_NONE ) {
    sprintf( err_buffer, "(planner.cpp) activate() failed calling control(state,command) for simtime %f\n", simtime.seconds );
    error_log.write( err_buffer );
    return ERROR_FAILED;
  }

  // publish the initial command message
  command.header.type = MSG_COMMAND;
  if( publish( command ) != ERROR_NONE ) {
    sprintf( err_buffer, "(planner.cpp) activate() failed calling publish(command) for simtime %f\n", simtime.seconds );
    error_log.write( err_buffer );
    return ERROR_FAILED;
  }

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Initialization sequence for a standalone controller
void planner_c::init( void ) {

  // connect to the error log
  // Note: error log is created by the coordinator
  error_log = log_c( FD_ERROR_LOG );
  if( error_log.open( ) != LOG_ERROR_NONE ) {
    // Note:  this is not really necessary.  If coordinator launched properly, this should never happen
    printf( "(planner.cpp) ERROR: init() failed calling log_c.open() on FD_ERROR_LOG\nController Exiting\n" );
    exit( 1 );
  }

  // connect to the shared message buffer BEFORE attempting any IPC
  // Note: the message buffer is created by the coordinator before the controller
  // is launched
  //msg_buffer = sharedbuffer_c( BUFFER_NAME, false );
  msg_buffer = sharedbuffer_c( "/amsgbuffer", false );
  if( msg_buffer.open( ) != BUFFER_ERR_NONE ) {
    sprintf( err_buffer, "(planner.cpp) init() failed calling sharedbuffer_c.open(...,false)\nController Exiting\n" );
    printf( "%s", err_buffer );
    error_log.write( err_buffer );
    error_log.close( );
    exit( 1 );
  }
}

//-----------------------------------------------------------------------------

void planner_c::shutdown( void ) {
  printf( "Planner shutting down\n" );

  msg_buffer.close( );  // TODO : figure out a way to force a clean up
}


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
/*
bool planner_c::plan_for_prey( const double& t, const pp_state_c& q, ship_command_c& u ) {
  std::vector<double> pred_q, prey_q, prey_u;

  pred_q = q.pred_vector();
  prey_q = q.prey_vector();

  prey_command( pred_q, prey_q, prey_u, adversary, self, t );

  u = ship_command_c( prey_u );

  return true;
}
*/
//-----------------------------------------------------------------------------
bool planner_c::plan_for_predator( const double& t, const pp_state_c& q, ship_command_c& u ) {

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

//-----------------------------------------------------------------------------
/// Allocates a predator/prey scenario control sampler
ompl::control::DirectedControlSamplerPtr planner_c::allocate_control_sampler( const ompl::control::SpaceInformation* si ) {
  int k = 1;
  //return ompl::control::DirectedControlSamplerPtr( new control_sampler_c( si, self, adversary, &ship_c::best_control, k ) );
  return ompl::control::DirectedControlSamplerPtr( new control_sampler_c( si, self, adversary, best_control, k ) );
}


//-----------------------------------------------------------------------------
// OMPL Planning
//-----------------------------------------------------------------------------
/// Plans motion for the ship using an rrt planner
bool planner_c::plan_rrt( ship_p self, ship_p adversary, const pp_state_c& q, const pp_state_c& q_bounds, ship_command_c& u, const ship_command_c& u_bounds ) {

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
bool planner_c::is_state_valid(const ompl::control::SpaceInformation *si, const ompl::base::State *state) {

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

//-----------------------------------------------------------------------------

