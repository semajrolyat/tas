#include <ship.h>

#include "state_propagator.h"
#include "directed_control_sampler.h"

using boost::shared_ptr;
using namespace Ravelin;

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
GZ_REGISTER_MODEL_PLUGIN( ship_c )

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
ship_c::ship_c( void ) { 

}

//-----------------------------------------------------------------------------
ship_c::ship_c( const ompl::base::StateSpace *_statespace ) : statespace(_statespace) {

}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
ship_c::~ship_c( void ) {
    gazebo::event::Events::DisconnectWorldUpdateBegin( this->updateConnection );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void ship_c::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  const unsigned X = 0, Y = 1, Z = 2;

  model = _model;
  world = _model->GetWorld();
  space = space_c( _model->GetWorld() );

  body = model->GetLink("body");
  if( !body ) {
    gzerr << "Unable to find link: body\n";
    return;
  }

  GAUSSIAN_MEAN = 0.0;
  GAUSSIAN_VARIANCE = 35.0;
  GAUSSIAN_STDDEV = sqrt( GAUSSIAN_VARIANCE );

  // get inertial properties from Gazebo
  gazebo::physics::InertialPtr inertial = body->GetInertial();

  // setup mass
  _inertial.m = inertial->GetMass();

  // setup moment of inertia
  gazebo::math::Vector3 pm = inertial->GetPrincipalMoments();
  _inertial.J(X,X) = pm.x;  _inertial.J(X,Y) = 0.0;  _inertial.J(X,Z) = 0.0;
  _inertial.J(Y,X) = 0.0;   _inertial.J(Y,Y) = pm.y; _inertial.J(Y,Z) = 0.0;
  _inertial.J(Z,X) = 0.0;   _inertial.J(Z,Y) = 0.0;  _inertial.J(Z,Z) = pm.z;

  // NOTE: center-of-mass is assumed to be (0,0,0) relative to the body frame

  //---------------------------------------------------------------------------
  // Call Init Function (Note: target function subject to inheritance)
  //---------------------------------------------------------------------------
  init();
}

//-----------------------------------------------------------------------------
void ship_c::Update( ) {

  // Note: we only do spatial query on first pass (and not on initialization)
  // because it is nondeterministic (or treated as such) when the world gets 
  // loaded.
  static bool first_update = true;
  if( first_update ) {

    // Note: following may not be tight fit if the ship begins unaligned to world
    // and error may propagate through future collision queries
    ship_frame_bb = aabb();    

/*
    // get the reference to the spatial bound
    gazebo::physics::ModelPtr space = world->GetModel("pen");
    if( !space ) 
      gzerr << "Unable to find model: pen\n";
    // compute & cache the spatial bound aabb

    // Space is empty inside the boundary, but the boundary is made up of thin walls not just a cube.
    gazebo::physics::Link_V space_links = space->GetLinks();
    gazebo::math::Vector3 c( 0.0, 0.0, 0.0 );
    gazebo::math::Vector3 e( 0.0, 0.0, 0.0 );
    for( unsigned i = 0; i < space_links.size(); i++ ) {
      gazebo::physics::LinkPtr link = space_links[i];
      gazebo::math::Box gzbb = link->GetBoundingBox();
      c += gzbb.GetCenter();
      // Note:: following assumes centered at (0,0,0) and symmetric.  Dirty but works.
      e = gazebo::math::Vector3( std::max(gzbb.GetCenter().x, e.x), std::max(gzbb.GetCenter().y, e.y),std::max(gzbb.GetCenter().z, e.z) );
    }
    spatial_bound = aabb_c( c, e );
*/
    space.read();
    spatial_bound = space.bounds;

    //std::cout << "space: " << spatial_bound << std::endl;

/*
    // get the reference to the obstacles    
    gazebo::physics::ModelPtr maze = world->GetModel("maze");
    if( !maze ) 
      gzerr << "Unable to find model: maze\n";
    
    // compute and cache the static world aabb's
    gazebo::physics::Link_V maze_links = maze->GetLinks();
    for( unsigned i = 0; i < maze_links.size(); i++ ) {
      gazebo::physics::LinkPtr link = maze_links[i];
      // Note: following may need to query GetCollisionBox instead
      gazebo::math::Box gzbb = link->GetBoundingBox();
      gazebo::math::Vector3 c = gzbb.GetCenter();
      gazebo::math::Vector3 e = gzbb.GetSize() / 2;

      aabb_c obstacle( c, e );
      obstacles.push_back( obstacle );
    }
*/
    first_update = false;
  }

  time = world->GetSimTime().Double() - time_start;
  dtime = time - time_last;

  sense( );

  if( !stopped && !plan( ) ) {
    //std::cout << "planning failed\n";
    time_last = time;
  }
  
  if( stopped ) 
    stop( );
  else
    act( );

  //test_intersection_detection(); 

  time_last = time;
}

//-----------------------------------------------------------------------------
/*
void ship_c::Reset( ) {

}
*/

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void ship_c::init( void ) {
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind( &ship_c::Update, this ) );

  audit_file_planned_commands = "ship_planned_commands.txt";
  audit_file_planned_states = "ship_planned_states.txt";
  audit_file_actual_commands = "ship_actual_commands.txt";
  audit_file_actual_states = "ship_actual_states.txt";
  audit_file_fb_commands = "ship_fb_commands.txt";
  audit_file_ff_commands = "ship_ff_commands.txt";
  audit_file_interp_states = "ship_interp_states.txt";
  audit_file_fb_error = "ship_fb_error_states.txt";
  audit_file_control_values = "ship_control_values.txt";

  write_command_audit_header( audit_file_planned_commands );
  write_state_audit_header( audit_file_planned_states );
  write_command_audit_header( audit_file_actual_commands );
  write_state_audit_header( audit_file_actual_states );

  write_state_audit_header( audit_file_fb_error );
/*
  //write_command_audit_header( audit_file_ff_commands );
  //write_command_audit_header( audit_file_fb_commands );
  //write_state_audit_header( audit_file_interp_states );
  //write_command_audit_header( audit_file_control_values );
*/
  std::cerr << "Ship Controller Plugin Started" << std::endl;
/*
  // initial test state
  gazebo::math::Vector3 p = gazebo::math::Vector3( 0.0, 0.0, 4.0 );  // for ompl case
  gazebo::math::Quaternion r = gazebo::math::Quaternion(0.0, 0.0, 0.0);
  model->SetWorldPose( gazebo::math::Pose( p, r ) );
*/
  stopped = false;

  time_start = world->GetSimTime().Double();
  time_last = time_start;

}

//-----------------------------------------------------------------------------
void ship_c::sense( void ) {

}

//-----------------------------------------------------------------------------
bool ship_c::plan( void ) {

  static bool has_planned = false;
///*
  ship_state_c predator( gazebo::math::Vector3(0,0,25), gazebo::math::Quaternion(1,0,0,0), gazebo::math::Vector3(0,0,0), gazebo::math::Vector3(0,0,0) );
  ship_state_c prey = state();

  std::vector<double> pred_state = predator.as_vector();
  std::vector<double> prey_state = prey.as_vector();
  std::vector<double> prey_u;
  prey_command( pred_state, prey_state, prey_u );

  command_current = ship_command_c( prey_u );
//*/
/*
  if( !has_planned ) {
    if( !plan_rrt() ) return false;
    has_planned = true;

    write_audit_data( audit_file_planned_states, states_desired );
    write_audit_data( audit_file_planned_commands, commands_desired );
  }

  if( command_current_duration <= 0.0 ) {

    if( commands_desired.size() == 0 ) {
      if( !stopped ) std::cout << "Stopping\n";
      stopped = true;
      return false;
    }

    std::pair<double, ship_command_c> u_pair = commands_desired.front();
    commands_desired.erase( commands_desired.begin() );

    command_current_duration = u_pair.first;
    command_current = u_pair.second;
  }

  desired_state_duration += dtime;
  if( desired_state_duration >= PLANNER_STEP_SIZE ) {
    update_desired_state( );
  }
*/

  return true;
}


//-----------------------------------------------------------------------------
void ship_c::act( void ) {
  command_current_duration -= dtime;

  // treat command_current as ff
  ship_command_c ff_command = command_current;

  // compute fb
  ship_command_c fb_command = compute_feedback();

  // sum ff and fb
  ship_command_c cmd = ff_command + fb_command;
  //ship_command_c cmd = ff_command;
  //ship_command_c cmd = fb_command;

  // control based on summed command
  control( cmd );

  std::pair<double,ship_state_c> actual_state = std::make_pair( time, state() );
  write_audit_datum( audit_file_actual_states, actual_state );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
/// Build an axis aligned bounding box of the ship based on state vector
/// TODO : Evan please verify.  Computation of extens doesn't sound right
aabb_c ship_c::aabb( const std::vector<double>& q ) {

  //pull the center of the ship from the state vector
  Origin3d c( q[0], q[1], q[2] );
  //rotate the ship frame aabb
  Quatd e( q[3], q[4], q[5], q[6] );
  e.normalize();  // Just in case

  //recompute the extens
  // extract the ship frame extens
  Origin3d sfext( ship_frame_bb.extens.x, ship_frame_bb.extens.y, ship_frame_bb.extens.z );
  // compute extens in rotation frame
  Origin3d rfext = e * sfext;
  
  return aabb_c ( c, rfext );
}

//-----------------------------------------------------------------------------
/// Query the current axis aligned bounding box of the ship
aabb_c ship_c::aabb( void ) {
  // Note: following may need to query GetCollisionBox instead
  gazebo::math::Box gzbb = model->GetBoundingBox();
  gazebo::math::Vector3 c = gzbb.GetCenter();
  gazebo::math::Vector3 e = gzbb.GetSize() / 2;
  return aabb_c ( c, e );
}

//-----------------------------------------------------------------------------
/// Queries whether the ship intersects any obstacle in the world
bool ship_c::intersects_any_obstacle( const aabb_c& mybb, aabb_c& obstacle ) {
  for( unsigned i = 0; i < obstacles.size(); i++ ) {
    if( aabb_c::intersects( mybb, obstacles[i] ) ) {
      obstacle = obstacles[i];
      return true;
    }
  }
  return false;
}

//-----------------------------------------------------------------------------
/// Queries whether the ship remains inside the world boundary
bool ship_c::intersects_world_bounds( const aabb_c& mybb ) {
  return !aabb_c::intersects( mybb, spatial_bound );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void ship_c::update_desired_state( void ) {
  if( stopped ) return;

  std::pair<double, ship_state_c> q = states_desired.front();
  desired_state_duration = 0.0;

  desired_state_0 = q.second;
  states_desired.erase( states_desired.begin() );

  std::pair<double, ship_state_c> q1 = states_desired.front();
  desired_state_1 = q1.second;
  // if front == end path is done!  Candidate trigger for replanning
  state_step = 0;
}

//------------------------------------------------------------------------------
void ship_c::compute_desired_state( ship_state_c& result ) {
/*
  // interpolation required for the test case
  result = interpolate_linear( desired_state_0, desired_state_1, PLANNER_STEP_SIZE, ++state_step );
*/
///*
  // the command has already been computed as current_command

  double DT = dtime;

  // use forward dynamics to compute the change in state applying the command causes
  std::vector<double> u = command_current.as_vector();
  std::vector<double> q = state().as_vector();
  std::vector<double> dq;
  ode( q, u, dq );

  // integrate the change in state with the current state to get the desired state
  for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
    q[i] += dq[i] * DT;
  }
  // return the desired state

  result = ship_state_c( q );

  std::pair<double,ship_command_c> up = std::make_pair( DT, command_current );
  write_audit_datum( audit_file_planned_commands, up );
  std::pair<double,ship_state_c> qp = std::make_pair( time, result );
  write_audit_datum( audit_file_planned_states, qp );

//*/
}

//------------------------------------------------------------------------------
ship_state_c ship_c::interpolate_linear( const ship_state_c& q0, const ship_state_c& qf, const double& deltat, const int& step ) const {
  ship_state_c _qf( qf );
  ship_state_c _q0( q0 );
  const double t = deltat * (double)step;
  ship_state_c dq = _qf - _q0;
  ship_state_c tdq = t * dq;

  return tdq + _q0;
}

//-----------------------------------------------------------------------------
ship_command_c ship_c::compute_feedback( void ) {
  //ship_state_c qx = interpolate_linear( desired_state_0, desired_state_1, PLANNER_STEP_SIZE, ++state_step );
  //std::cout << "qx:" << qx << std::endl; 

  ship_state_c qx;
  compute_desired_state( qx );

  // normalize qx's quaternion
  Quatd ex(qx.value(3), qx.value(4), qx.value(5), qx.value(6));
  ex.normalize();
  qx(3) = ex.x;
  qx(4) = ex.y;
  qx(5) = ex.z;
  qx(6) = ex.w;

  // get quaternion from current state
  ship_state_c q = state();
  Quatd e(q.value(3), q.value(4), q.value(5), q.value(6));

  ship_state_c deltaq = qx - q;

  const double K_pl = 100.0;
  const double K_vl = 10.0;

  const double K_pr = 10.0;
  const double K_vr = 1.0;

  Vector3d pos_err( deltaq.value(0), deltaq.value(1), deltaq.value(2) );
  Vector3d vel_err( deltaq.value(7), deltaq.value(8), deltaq.value(9) );

  Quatd rot_err( deltaq.value(3), deltaq.value(4), deltaq.value(5), deltaq.value(6) );
  Vector3d rotv_err( deltaq.value(10), deltaq.value(11), deltaq.value(12) );

  Vector3d F = K_pl * pos_err + K_vl * vel_err;

  // convert differential quaternion to angular velocity
  Vector3d rotp = Quatd::to_omega(e, rot_err);

  // multiply by positional gain
  rotp *= K_pr;

  // compute the derivative error of orientation
  Vector3d rotd = rotv_err;

  // multiply by derivative gain
  rotd *= K_vr;

  // compute the combined control
  Vector3d tau = rotp + rotd;

  // build the command
  ship_command_c u;
  u.force( F );
  u.torque( tau );

  return u;
}
 
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void ship_c::control( ship_command_c& u ) {
  control_position( u );
  control_rotation( u );
}

//-----------------------------------------------------------------------------
void ship_c::control_position( ship_command_c& u ) {
  // clear the accumulators
  gazebo::math::Vector3 f = -body->GetRelativeForce();
  body->AddRelativeForce( f );

  // add the command force
  body->AddForce( u.force() );
}

//-----------------------------------------------------------------------------
void ship_c::control_rotation( ship_command_c& u ) {
  // clear the accumulators
  gazebo::math::Vector3 t = -body->GetRelativeTorque();
  body->AddRelativeTorque( t );

  // add the command torque
  body->AddTorque( u.torque() );
}

//-----------------------------------------------------------------------------
void ship_c::stop( void ) {
  ship_command_c cmd;
  cmd.force( gazebo::math::Vector3( 0.0, 0.0, 0.0) );
  cmd.torque( gazebo::math::Vector3( 0.0, 0.0, 0.0) );

  control( cmd );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
ship_state_c ship_c::state( void ) {
  gazebo::math::Vector3 pos = model->GetWorldPose().pos;
  gazebo::math::Quaternion rot = model->GetWorldPose().rot;
  gazebo::math::Vector3 dpos = model->GetWorldLinearVel();
  gazebo::math::Vector3 drot = model->GetWorldAngularVel();

  ship_state_c q( pos, rot, dpos, drot );

  return q;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool ship_c::plan_simple( void ) {
  // Note: state only.  Valid only for fb testing
  
  double px, py, pz;
  double rx;
  //double rx, ry, rz, rw;
  // sinusoidal
  double dt = 2*PI / 100.0;
  PLANNER_STEP_SIZE = dt;

  gazebo::math::Vector3 x0( 0.0, 0.0, 0.0 );

  for( double t = 0.0; t <= 2 * PI; t += dt ) {
    px = sin(t);
    py = sin(t);
    pz = sin(t);

    rx = sin(t);

    gazebo::math::Vector3 pos( px, py, pz );
    gazebo::math::Quaternion rot( 1.0, rx, 0.0, 0.0 );
    gazebo::math::Vector3 dpos = (pos - x0) / dt;
    gazebo::math::Vector3 drot( cos(t), 0.0, 0.0 );
    
    ship_state_c q( pos, rot, dpos, drot );

    std::pair<double,ship_state_c> qpr = std::make_pair( t, q );
    states_desired.push_back( qpr );

    x0 = pos;
  }

  // NOTE: commands are bogus at this point
  unsigned int control_count = states_desired.size();
  for( unsigned int i = 0; i < control_count; i++ ) {
    // bogus forces and torques
    ship_command_c u( gazebo::math::Vector3(0.0,0.0,0.0), gazebo::math::Vector3(0.0,0.0,0.0) );
    std::pair<double,ship_command_c> upr = std::make_pair( dt, u );
    commands_desired.push_back( upr );
  }
  update_desired_state( );

  command_current_duration = 0.0;

  time_start = world->GetSimTime().Double();
  time_last = time_start;

  return true;
}

//-----------------------------------------------------------------------------
// OMPL
//-----------------------------------------------------------------------------
bool ship_c::plan_rrt( void ) {

  const double MAX_PLANNING_TIME = 10.0;
  PLANNER_STEP_SIZE = .1;
  //const double GOAL_THRESHOLD = 1e-5; 
  const double MAX_COMMAND = 100.0;

  /// construct the state space we are planning in
  ompl::base::StateSpacePtr space( new ompl::base::RealVectorStateSpace( ship_state_c::size() ) );
  statespace = space.get();

  // Set the bounds for the state space
  ompl::base::RealVectorBounds bounds( ship_state_c::size() );
  for (unsigned i = 0; i < 3; i++)  {
    // x, y, z extens
    bounds.setLow(i, -25.0);
    bounds.setHigh(i, 25.0);
  }
  for (unsigned i=3; i< 7; i++)
  {
    // quaternion extents 
    bounds.setLow(i, -1.0);
    bounds.setHigh(i, 1.0);
  }
  for (unsigned i = 7; i < ship_state_c::size(); i++)  {
    // all other extens
    bounds.setLow(i, -1e1);
    bounds.setHigh(i, 1e1);
  }
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

  // create a control space
  ompl::control::ControlSpacePtr cspace(new control_space_c(space));

  // set the bounds for the control space
  ompl::base::RealVectorBounds cbounds( ship_command_c::size() );
  cbounds.setLow( -MAX_COMMAND );
  cbounds.setHigh( MAX_COMMAND );
  cspace->as<control_space_c>()->setBounds(cbounds);

  // define a simple setup class
  ompl::control::SimpleSetup ss(cspace);

  ompl::control::SpaceInformationPtr si( ss.getSpaceInformation() );

  si->setDirectedControlSamplerAllocator( boost::bind(&ship_c::allocate_directed_control_sampler, this, _1) );

  ompl::base::ProblemDefinition pdef(si);

  /// set state validity checking for this space
  ss.setStateValidityChecker(boost::bind(&ship_c::is_state_valid, this, si.get(), _1 ));

  /// set the propagation routine for this space
  ss.setStatePropagator( ompl::control::StatePropagatorPtr( new state_propagator_c( si, this ) ) );

  // create initial and goal states
  std::vector<double> initial( ship_state_c::size() ), target( ship_state_c::size() );

/*
  gazebo::math::Vector3 pos0( 0.0, 4.0, 0.0 );
  gazebo::math::Quaternion rot0( 0.0, 0.0, 0.0 );
  gazebo::math::Vector3 dpos0( 0.0, 0.0, 0.0 );
  gazebo::math::Vector3 drot0( 0.0, 0.0, 0.0 );
*/
  gazebo::math::Vector3 posf( 0.0, 0.0, 0.0 );
  gazebo::math::Quaternion rotf( PI, 0.0, 0.0 );
  gazebo::math::Vector3 dposf( 0.0, 0.0, 0.0 );
  gazebo::math::Vector3 drotf( 0.0, 0.0, 0.0 );

  ship_state_c state0, statef;

  state0 = state();
  std::cout << state0 << std::endl;

  statef.position( posf );
  statef.rotation( rotf );
  statef.dposition( dposf );
  statef.drotation( drotf );

  for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
    initial[i] = state0(i);
    target[i] = statef(i);
  }
  
  /// create a start state
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space);
  to_state( space.get(), initial, start.get() );

  /// create a  goal state; use the hard way to set the elements
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space);
  to_state( space.get(), target, goal.get() );

  boost::shared_ptr <ship_goal_c> pgoal( new ship_goal_c(si) );
  pgoal->setState( goal );

  /// set the start and goal states
  ss.setStartState( start );
  ss.setGoal( pgoal );

  ompl::control::RRT* rrt = new ompl::control::RRT(si);
  rrt->setGoalBias( 0.05 );
  ompl::base::PlannerPtr planner( rrt );

  ss.setPlanner( planner );

  ss.getSpaceInformation()->setPropagationStepSize( PLANNER_STEP_SIZE );
  //ss.getSpaceInformation()->setMinMaxControlDuration(1, 10);
  ss.setup();
  static_cast<state_propagator_c*>(ss.getStatePropagator().get())->setIntegrationTimeStep(ss.getSpaceInformation()->getPropagationStepSize());

  ompl::base::PlannerStatus solved;
  solved = ss.solve( MAX_PLANNING_TIME );

  if (solved) {

    std::cout << "Found solution:" << std::endl;

    ompl::geometric::PathGeometric path = ss.getSolutionPath().asGeometric();
    unsigned int state_count = path.getStateCount();
///*
    const double EPSILON = 1e-4;
    // check that final state is very close to goal state
    ship_state_c state_plan_end( statespace, path.getState( state_count - 1 ) );

    if( fabs( state_plan_end(0) - statef(0) ) > EPSILON || fabs( state_plan_end(1) - statef(1) ) > EPSILON || fabs( state_plan_end(2) - statef(2) ) > EPSILON ) {
      std::cout << "Planning Failed: final state diverged significantly from goal state\n";
      if( fabs( state_plan_end(0) - statef(0) ) > EPSILON ) {
        std::cout << "x diff: " << state_plan_end(0) - statef(0) << std::endl;
      }
      if( fabs( state_plan_end(1) - statef(1) ) > EPSILON ) {
        std::cout << "y diff: " << state_plan_end(1) - statef(1) << std::endl;
      }
      if( fabs( state_plan_end(2) - statef(2) ) > EPSILON ) {
        std::cout << "z diff: " << state_plan_end(2) - statef(2) << std::endl;
      }
      
      return false;
    }
//*/
    for( unsigned int i = 0; i < state_count; i++ ) {
      ship_state_c q( statespace, path.getState( i ) );
      double t = (double)i * PLANNER_STEP_SIZE;
      std::pair<double,ship_state_c> qp = std::make_pair( t, q );
      states_desired.push_back( qp );
    }

    unsigned int control_count = ss.getSolutionPath().getControlCount();
    for( unsigned int i = 0; i < control_count; i++ ) {
      ship_command_c u( ss.getSolutionPath().getControl(i) );
      double t = ss.getSolutionPath().getControlDuration( i );

      std::pair<double,ship_command_c> up = std::make_pair( t, u );
      commands_desired.push_back( up );
    }
    update_desired_state( );

    command_current_duration = 0.0;

    time_start = world->GetSimTime().Double();
    time_last = time_start;

    return true;
  } else {
    std::cout << "No solution found" << std::endl;
    return false;
  }
}

//-----------------------------------------------------------------------------
bool ship_c::is_state_valid(const ompl::control::SpaceInformation *si, const ompl::base::State *state) {

  std::vector<double> values( ship_state_c::size() );
  from_state( si->getStateSpace().get(), state, values );
 
  aabb_c bb = aabb( values );
  aabb_c obstacle;
  
  if( intersects_world_bounds( bb ) )
    return false;
  if( intersects_any_obstacle( bb, obstacle ) )
    return false;

  return true;
}

//-----------------------------------------------------------------------------
/// Ship ordinary differential equation (ODE) (Called by state propagator)
void ship_c::operator()( const ompl::base::State* ompl_q, const ompl::control::Control* ompl_u, std::vector<double>& dq, const ship_c* ship ) const {

  std::vector<double> q( ship_state_c::size() );
  from_state( statespace, ompl_q, q );

  std::vector<double> u( ship_command_c::size() );
  const double *_u = ompl_u->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
  for( unsigned i = 0; i < ship_command_c::size(); i++ ) {
    u[i] = _u[i];
  }

  ship->ode( q, u, dq );
}

//-----------------------------------------------------------------------------
void ship_c::ode( const std::vector<double>& q, const std::vector<double>& u, std::vector<double>& dq ) const {

  dq.resize( ship_state_c::size() );

  // - Newton-Euler -

  // setup the ship quaternion
  Quatd e(q[3], q[4], q[5], q[6]);

  // setup the ship's pose in a center-of-mass frame
  shared_ptr<Pose3d> P(new Pose3d);
  P->x = Origin3d(q[0], q[1], q[2]);
  P->q.set_identity();

  // the inertia is setup in the body frame
  shared_ptr<Pose3d> Pi(new Pose3d);
  Pi->x = P->x;
  Pi->q = e; 

  // get the linear velocity 
  Vector3d omega(q[10], q[11], q[12], P);
  SVelocityd vel(q[10], q[11], q[12], q[7], q[8], q[9], P);
  Quatd edot = Quatd::deriv(e, omega);

  // get the force
  SForced force(u[0], u[1], u[2], u[3], u[4], u[5], P);

  // transform the inertia to frame P
  SpatialRBInertiad inertial = _inertial;
  inertial.pose = Pi;
  SpatialRBInertiad J = Pose3d::transform(P, inertial);

  // get the acceleration
  SAcceld acc = J.inverse_mult(force - vel.cross(J * vel));
  Vector3d xdd = acc.get_linear();
  Vector3d alpha = acc.get_angular();

  // update dq (linear velocity)
  dq[0] = q[7];
  dq[1] = q[8];
  dq[2] = q[9];

  // update dq (quaternion derivative) 
  dq[3] = edot.x;
  dq[4] = edot.y;
  dq[5] = edot.z;
  dq[6] = edot.w; 
  // Note: Renormalize? => Not now! Only After integrating!

  // update dq (linear acceleration)
  dq[7] = xdd[0];
  dq[8] = xdd[1];
  dq[9] = xdd[2];

  // update dq (angular acceleration)
  dq[10] = alpha[0];
  dq[11] = alpha[1];
  dq[12] = alpha[2];

}

//-----------------------------------------------------------------------------
/// Compute the inverse dynamics of the ship
void ship_c::inv_dyn( const std::vector<double>& q, const std::vector<double>& qdot_des, std::vector<double>& u) const {

  // - Newton-Euler -

  // setup the ship quaternion
  Quatd e(q[3], q[4], q[5], q[6]);

  // setup the ship's pose in a center-of-mass frame
  shared_ptr<Pose3d> P(new Pose3d);
  P->x = Origin3d(q[0], q[1], q[2]);
  P->q.set_identity();

  // the inertia is setup in the body frame
  shared_ptr<Pose3d> Pi(new Pose3d);
  Pi->x = P->x;
  Pi->q = e; 

  // get the linear velocity 
  Vector3d omega(q[10], q[11], q[12], P);
  SVelocityd vel(q[10], q[11], q[12], q[7], q[8], q[9], P);
  //Quatd edot = Quatd::deriv(e, omega);

  // get desired linear acceleration 
  Vector3d xdd(P);
  xdd[0] = qdot_des[7];
  xdd[1] = qdot_des[8];
  xdd[2] = qdot_des[9];

  // get desired angular acceleration 
  Vector3d alpha(P);
  alpha[0] = qdot_des[10];
  alpha[1] = qdot_des[11];
  alpha[2] = qdot_des[12];

  // setup the acceleration
  SAcceld acc(P);
  acc.set_linear(xdd);
  acc.set_angular(alpha);

  // transform the inertia to frame P
  SpatialRBInertiad inertial = _inertial; 
  inertial.pose = Pi;
  SpatialRBInertiad J = Pose3d::transform(P, inertial);

  // get the acceleration
  SForced pseudo = vel.cross(J*vel);
  SForced force = J*acc + pseudo;

  // store the forces necessary to achieve that acceleration
  u.resize(6);
  Vector3d f = force.get_force();
  Vector3d tau = force.get_torque();
  u[0] = f[0];
  u[1] = f[1];
  u[2] = f[2];
  u[3] = tau[0];  
  u[4] = tau[1];
  u[5] = tau[2];
}

//-----------------------------------------------------------------------------
/// Update the state of the ship after integration (Called by state propagator)
void ship_c::update( ompl::base::State* ompl_q, const std::vector<double>& dq ) const {
  static std::vector<double> q;

  q.resize( ship_state_c::size() );
  from_state( statespace, ompl_q, q );
  for( unsigned i=0; i< ship_state_c::size(); i++ )
    q[i] += dq[i];

  // Renormalize quaternion
  gazebo::math::Quaternion e( q[6], q[3], q[4], q[5] );
  e.Normalize();
  q[3] = e.x;
  q[4] = e.y;
  q[5] = e.z;
  q[6] = e.w;

  to_state( statespace, q, ompl_q );
  statespace->enforceBounds( ompl_q );
 
}

//------------------------------------------------------------------------------
ompl::control::DirectedControlSamplerPtr ship_c::allocate_directed_control_sampler( const ompl::control::SpaceInformation* si ) {
  int k = 1;
  return ompl::control::DirectedControlSamplerPtr( new directed_control_sampler_c( si, this, k ) );
}

//------------------------------------------------------------------------------
// Testing 
//------------------------------------------------------------------------------
/// Test whether intersection of aabb's works
void ship_c::test_intersection_detection( void ) {
  aabb_c mybb = aabb();
  aabb_c obstacle;
  if( intersects_any_obstacle( mybb, obstacle ) )
    std::cout << "obstacle intersection" << std::endl;
  if( intersects_world_bounds( mybb ) )
    std::cout << "world intersection" << std::endl;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// computes commands (forces) for if the ship is a prey
void ship_c::prey_command( const std::vector<double>& pred_state, const std::vector<double>& prey_state, std::vector<double>& prey_u ) {

  // open parameters
  const double FLEE_DIST = 1.0;

  const double MAX_FORCE = 1e-5;
  const double MAX_TORQUE = 1e-5;

  const double WEIGHT_PREDATOR_FORCE = 1.0;
  const double WEIGHT_COMBINED_FORCE = 1.0;

  // get the predator and prey positions
  Vector3d pred_x( pred_state[0], pred_state[1], pred_state[2] );
  Vector3d prey_x( prey_state[0], prey_state[1], prey_state[2] );

  // determine distance to the predator
  Vector3d pp_vec = prey_x - pred_x;
  double dist = pp_vec.norm();

  // case 1: (semi) random walk
  if( dist > FLEE_DIST ) {
    //std::cout << "pred:" << pred_x << ", prey:" << prey_x << std::endl;

    // determine a desired position/orientation and velocities
    // NOTE: desired orientation should likely point toward intended 
    // FOLLOW-UP: Not easy to compute orientation at this point.  Instead random 
    // use inverse dynamics to compute the command
    // NOTE: because of lack of enforcing acceleration constraints, computing 
    // command instead as random

    // compute the seed for the random number generator
    long TSCALE = 1000;
    long seed = (long)(time / dtime) / TSCALE;
    //std::cout << "seed:" << seed << ", dtime:" << dtime << ", time:" << time  << std::endl;

    // setup the random number generator
    gsl_rng * r =  gsl_rng_alloc( gsl_rng_default );
    if( r == NULL ) std::cout << "failed to initialize rng\n";
    gsl_rng_set( r, seed );

    // generate command values from gaussian
    const double SIGMA = GAUSSIAN_STDDEV;
    const double MU = GAUSSIAN_MEAN;
    std::vector<double> u( ship_command_c::size() );
    for( unsigned i = 0; i < ship_command_c::size(); i++ )
      if( i < 3 )
        u[i] = gsl_ran_gaussian( r, SIGMA ) + MU;
      else
        u[i] = 0;  // Note: not adding torque yet for debugging.  Also, collisions with world are adding rotation

    // free the random number generator
    gsl_rng_free( r );
 
    ship_command_c cmd( u );
    

    // NOTE: make sure command is limited to max force/torque
    // if the force exceeds the maximum force, limit the force to the max
    if( cmd.force().GetLength() < MAX_FORCE ) {
      gazebo::math::Vector3 f = cmd.force().Normalize() * MAX_FORCE;
      cmd.force( f );
    }
    // if the torque exceeds the maximum torque, limit the torque to the max
    if( cmd.torque().GetLength() < MAX_TORQUE ) {
      gazebo::math::Vector3 tau = cmd.torque().Normalize() * MAX_TORQUE;
      cmd.torque( tau );
    }

    Ravelin::Vector3d bos_force = boundary_force(Vector3d(prey_state[0],prey_state[1],prey_state[2]),Vector3d(prey_state[7],prey_state[8],prey_state[9]));
    Ravelin::Vector3d f( cmd.force().x, cmd.force().y, cmd.force().z );
    f += bos_force;
    cmd.force( f );

    prey_u = cmd.as_vector();

  } else {
    // case 2: flee behavior from predator

    // get the vector from the predator to the prey and normalize it
    pp_vec /= dist;

    // treat the predator to prey (p2p) vector as a repulsive force (i.e. pushing the prey)
    // scale the p2p force
    double mag_predator_force = repulsive_force( dist ) * WEIGHT_PREDATOR_FORCE;
    Ravelin::Vector3d p2p_force = pp_vec * mag_predator_force; 

    Ravelin::Vector3d bos_force = boundary_force(Vector3d(prey_state[0],prey_state[1],prey_state[2]),Vector3d(prey_state[7],prey_state[8],prey_state[9]));

    // add the two vectors together
    Ravelin::Vector3d force = p2p_force + bos_force;

    // scale the summed force
    force *= WEIGHT_COMBINED_FORCE;

    // use scaled summed force as the prey command.
    
    ship_command_c cmd;
    cmd.force( force );
    prey_u = cmd.as_vector();
    

    // ...

    // determine a pose that points toward the intended location (we'll use
    // an 'up' vector to eliminate ambiguity)

    // ...
  }
}

//------------------------------------------------------------------------------
Ravelin::Vector3d ship_c::boundary_force( const Ravelin::Vector3d& pos, const Ravelin::Vector3d& vel ) {
  const double WEIGHT_BOUNDARY_FORCE = 1.0;

  // compute the repulsive force from the boundary of space (bos)
  Ravelin::Vector3d bos_force(0,0,0);
  for( unsigned i = 0; i < space.planes.size(); i++ ) {
    double t;
    Ravelin::Vector3d intersect;
    if(space.planes[i].ray_intersection( pos, vel, t, intersect )) {
      // Note: this will almost always give three planes, so test whether the 
      // intersection point lies inside the bounding box.

      //Question Evan: Is there frame dependency in the rotation of the velocity vector?
      if( aabb_c::inside( intersect, space.bounds ) ) {
        Ravelin::Vector3d v = intersect - pos;
        double dist_to_plane = v.norm();
        double repulsion = repulsive_force( dist_to_plane );
        bos_force += space.planes[i].normal * repulsion;
        std::cout << "dist: " << dist_to_plane << ", f:" << bos_force << std::endl;
      }
    }
  }
  // scale the bos force
  bos_force *= WEIGHT_BOUNDARY_FORCE;
  return bos_force; 
}

//------------------------------------------------------------------------------
// computes a repulsive force for a given distance
double ship_c::repulsive_force( double dist ) {
  const double ALPHA = 1e1;  // this constant will likely need to be changed

  // logarithmic function will produce negative values for inputs in [0,1] and
  // positive values for inputs > 1
  return (std::log(ALPHA*dist) <= 0.0) ? -std::log(ALPHA*dist) : 0.0;
}

//------------------------------------------------------------------------------
