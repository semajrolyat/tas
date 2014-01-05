#include <ship.h>

//-----------------------------------------------------------------------------

using boost::shared_ptr;
using namespace Ravelin;

//-----------------------------------------------------------------------------

GZ_REGISTER_MODEL_PLUGIN( ship_c )

//-----------------------------------------------------------------------------
// Constructors
//-----------------------------------------------------------------------------
/// Default Constructor
ship_c::ship_c( void ) { 
  capture = false;

}

//-----------------------------------------------------------------------------
/// Constructs a ship reference based on player type for predator/prey scenario
ship_c::ship_c( ship_c* owner, const player_type_e& player_type ) {
  if( player_type == PREY ) {
    ADVERSARY_TYPE = PREDATOR;
  } else {
    ADVERSARY_TYPE = PREY;
  }
  capture = false;
  CAPTURE_DISTANCE = owner->CAPTURE_DISTANCE;
  FLEE_DISTANCE = owner->FLEE_DISTANCE;
  PREY_MAX_FORCE = owner->PREY_MAX_FORCE;
  PREY_MAX_TORQUE = owner->PREY_MAX_TORQUE;
  PREY_PREDATOR_FORCE_WEIGHT = owner->PREY_PREDATOR_FORCE_WEIGHT;
  PREY_COMBINED_FORCE_WEIGHT = owner->PREY_COMBINED_FORCE_WEIGHT;
  PREY_BOUNDARY_FORCE_WEIGHT = owner->PREY_BOUNDARY_FORCE_WEIGHT;

  PLANNER_STEP_SIZE = owner->PLANNER_STEP_SIZE;
  PLANNER_MAX_PLANNING_TIME = owner->PLANNER_MAX_PLANNING_TIME;
  PLANNER_MAX_DERIVATIVE = owner->PLANNER_MAX_DERIVATIVE;
  PLANNER_MAX_FORCE = owner->PLANNER_MAX_FORCE;
  PLANNER_GOAL_BIAS = owner->PLANNER_GOAL_BIAS;

  FEEDBACK_GAIN_PROPORTIONAL_POSITION = owner->FEEDBACK_GAIN_PROPORTIONAL_POSITION;
  FEEDBACK_GAIN_DERIVATIVE_POSITION = owner->FEEDBACK_GAIN_DERIVATIVE_POSITION;
  FEEDBACK_GAIN_PROPORTIONAL_ROTATION = owner->FEEDBACK_GAIN_PROPORTIONAL_ROTATION;
  FEEDBACK_GAIN_DERIVATIVE_ROTATION = owner->FEEDBACK_GAIN_DERIVATIVE_ROTATION;

  DRAG = owner->DRAG;
  REPULSIVE_FORCE_ALPHA = owner->REPULSIVE_FORCE_ALPHA;

  GAUSSIAN_MEAN = owner->GAUSSIAN_MEAN;
  GAUSSIAN_VARIANCE = owner->GAUSSIAN_VARIANCE;
  GAUSSIAN_STDDEV = sqrt(GAUSSIAN_VARIANCE);
}

//-----------------------------------------------------------------------------
// Destructor
//-----------------------------------------------------------------------------
ship_c::~ship_c( void ) {
    gazebo::event::Events::DisconnectWorldUpdateBegin( this->updateConnection );
}

//-----------------------------------------------------------------------------
// Gazebo Plugin Interface
//-----------------------------------------------------------------------------
/// Gazebo Load callback defined in ModelPlugin
// Note: the system has not finished loading up.  Indeterminent what models are
// loaded when.  Ideally we validate everything now, but it can't be done if
// the shared memory of the world is incomplete.  Will therefore only validate
// self in Load and in the first update, will validate everything else
void ship_c::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  first_update = true;

  self = ship_p( this ); 

  // Query what model this controller is handling
  std::string name = _model->GetName();
  if( name == "predator" || name == "prey" ) {
    // This ship is either predator or prey, so play the pursuit game
    GAME_TYPE = PURSUIT;
    if( name == "predator" ) {
      // This ship is predator and the other is prey
      PLAYER_TYPE = PREDATOR;
    } else if( name == "prey" ) {
      // This ship is prey and the other is predator
      PLAYER_TYPE = PREY;
    }
    if( !read_model( _model ) ) {
      gzerr << "Failed to read (" << name << ") model in " << name << "\n";
      return;
    }
    controller = ship_controller_c( body );
  } else {
    // bad configuration
    GAME_TYPE = UNDEFINED;
    gzerr << "Bad Configuration\n";
    return;
  }

  capture = false;

  CAPTURE_DISTANCE = 1.0;
  FLEE_DISTANCE = 4.0;
  PREY_MAX_FORCE = 1e-5;
  PREY_MAX_TORQUE = 1e-5;
  PREY_PREDATOR_FORCE_WEIGHT = 1.0;
  PREY_COMBINED_FORCE_WEIGHT = 1.0;
  PREY_BOUNDARY_FORCE_WEIGHT = 1.0;

  PLANNER_STEP_SIZE = 0.1;
  PLANNER_MAX_PLANNING_TIME = 0.1;
  PLANNER_MAX_DERIVATIVE = 2e1;
  PLANNER_MAX_FORCE = 1e2;
  PLANNER_GOAL_BIAS = 0.05;

  FEEDBACK_GAIN_PROPORTIONAL_POSITION = 100.0;
  FEEDBACK_GAIN_DERIVATIVE_POSITION = 10.0;
  FEEDBACK_GAIN_PROPORTIONAL_ROTATION = 10.0;
  FEEDBACK_GAIN_DERIVATIVE_ROTATION = 1.0;

  DRAG = 0.1;
  REPULSIVE_FORCE_ALPHA = 1e1;

  GAUSSIAN_MEAN = 0.0;
  GAUSSIAN_VARIANCE = 35.0;
  GAUSSIAN_STDDEV = sqrt(GAUSSIAN_VARIANCE);

  //---------------------------------------------------------------------------
  // Call Init Function (Note: target function subject to inheritance)
  //---------------------------------------------------------------------------
  init();
}

//-----------------------------------------------------------------------------
/// Gazebo callback on each update
void ship_c::Update( ) {

  // Note: See Note for Load(...) as to why validation is continued here on first update
  if( first_update ) {
    // Read world information
    if( !read_world() ) {
      // Failed World Validation
        gzerr << "Failed to read world information.  No guarantees on sim\n";
    }

    // Validate the game
    if( GAME_TYPE == PURSUIT ) {
      // Read the adversary information
      if( !read_adversary() ) {
        // Failed Validation
        gzerr << "Failed to read model information for adversary.  No guarantees on sim\n";
      }
      // Game appears generally valid.  Note: Some additional validation may be necessary

      if( PLAYER_TYPE == PREY ) 
        planner = planner_c( planner_c::WALK, &ode, &best_control, &prey_command, spatial_bound, self, adversary, PLANNER_STEP_SIZE, PLANNER_MAX_PLANNING_TIME, PLANNER_MAX_DERIVATIVE, PLANNER_MAX_FORCE, PLANNER_GOAL_BIAS );
      else 
        planner = planner_c( planner_c::RRT, &ode, &best_control, &prey_command, spatial_bound, self, adversary, PLANNER_STEP_SIZE, PLANNER_MAX_PLANNING_TIME, PLANNER_MAX_DERIVATIVE, PLANNER_MAX_FORCE, PLANNER_GOAL_BIAS );
      
    }
    first_update = false;
  }

  TIME = world->GetSimTime().Double() - TIME_START;
  DT = TIME - TIME_LAST;

  sense( state );

  if( !stopped && !plan( state, command ) ) {
    //std::cout << "planning failed\n";
    TIME_LAST = TIME;
  }
  
  if( stopped ) 
    stop( );
  else
    act( state, command );

  TIME_LAST = TIME;
}

//-----------------------------------------------------------------------------
/*
void ship_c::Reset( ) {

}
*/

//-----------------------------------------------------------------------------
// Robot Interface
//-----------------------------------------------------------------------------
/// Initialization of the robot
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

  stopped = false;

  TIME_START = world->GetSimTime().Double();
  TIME_LAST = TIME_START;

}

//-----------------------------------------------------------------------------
/// Encapsulates the sense phase of robot execution
void ship_c::sense( pp_state_c& q ) {

  if( GAME_TYPE != PURSUIT ) return;

  if( !capture ) {
    //std::vector<double> pred_q, prey_q;
    ship_state_c predator, prey;
    pred_sensor.sense( predator );
    prey_sensor.sense( prey );
    q = pp_state_c( predator, prey );
    capture = has_captured( state.pred_vector(), state.prey_vector() );
    if( capture ) {
      stopped = true;
      std::cout << "Prey is captured!" << std::endl; 
    } 
  }
}

//-----------------------------------------------------------------------------
/// Encapsulates the plan phase of robot execution
bool ship_c::plan( pp_state_c& q, ship_command_c& u ) {
  if( PLAYER_TYPE == PREY ) {
    return planner.plan_for_prey( TIME, q, u );
  } else if( PLAYER_TYPE == PREDATOR ) {
    return planner.plan_for_predator( TIME, q, u );
  }
  return false;
}


//-----------------------------------------------------------------------------
/// Encapsulates the act phase of robot execution
void ship_c::act( pp_state_c& q, ship_command_c& u ) {

  // treat command_current as ff
  ship_command_c u_ff = u;

  // compute fb
  ship_state_c qi; 
  my_state( q, qi );
  ship_command_c u_fb = compute_feedback( qi, u );

  // sum ff and fb
  u = u_ff + u_fb;
  //u = u_ff;
  //u = u_fb;

  // control based on summed command
  //control( u );
  controller.control( TIME, u );
}

//-----------------------------------------------------------------------------
// Utilities
//-----------------------------------------------------------------------------
/// Renormalize the quaternion component of a state vector
void ship_c::renormalize_state_quat( std::vector<double>& q )
{
  double qnorm = 0.0;
  for( unsigned i = 3; i < 7; i++ )
    qnorm += q[i] * q[i];
  qnorm = std::sqrt( qnorm );
  for( unsigned i = 3; i < 7; i++ )
    q[i] /= qnorm;
}

//-----------------------------------------------------------------------------
bool ship_c::has_captured( const std::vector<double>& pred_state, const std::vector<double>& prey_state ) {
  // get the predator and prey positions
  Vector3d pred_x( pred_state[0], pred_state[1], pred_state[2] );
  Vector3d prey_x( prey_state[0], prey_state[1], prey_state[2] );

  Vector3d pp_vec = prey_x - pred_x;
  double dist = pp_vec.norm();
  std::cout << "dist:" << dist << ", CAPTURE_DISTANCE:" << CAPTURE_DISTANCE << std::endl;

  if( dist <= CAPTURE_DISTANCE ) return true;
  return false;
}

//-----------------------------------------------------------------------------
// Gazebo Queries
//-----------------------------------------------------------------------------
/// Read a model definition from the gazebo system
bool ship_c::read_model( gazebo::physics::ModelPtr _model ) {
  const unsigned X = 0, Y = 1, Z = 2;

  model = _model;
  world = _model->GetWorld();
  space = space_c( _model->GetWorld() );

  body = _model->GetLink("body");
  if( !body ) {
    gzerr << "Unable to find link: body\n";
    return false;
  }

  // get inertial properties from Gazebo
  gazebo::physics::InertialPtr _inertial = body->GetInertial();

  // setup mass
  inertial.m = _inertial->GetMass();

  // setup moment of inertia
  gazebo::math::Vector3 pm = _inertial->GetPrincipalMoments();
  inertial.J(X,X) = pm.x;  inertial.J(X,Y) = 0.0;  inertial.J(X,Z) = 0.0;
  inertial.J(Y,X) = 0.0;   inertial.J(Y,Y) = pm.y; inertial.J(Y,Z) = 0.0;
  inertial.J(Z,X) = 0.0;   inertial.J(Z,Y) = 0.0;  inertial.J(Z,Z) = pm.z;

  // Note: center-of-mass is assumed to be (0,0,0) relative to the body frame

  // Note: aabb may not be tight fit if the ship begins unaligned to world
  // and error may propagate through future collision queries.
  ship_frame_bb = aabb();    

  return true;
}

//-----------------------------------------------------------------------------
/// Read the adversary definition from the gazebo system
bool ship_c::read_adversary( void ) {
  std::string my_name, adversary_name;
  gazebo::physics::ModelPtr adversary_model;

  if( PLAYER_TYPE == PREDATOR ) {
    // This ship is predator and the other is prey
    my_name = "predator";
    adversary_name = "prey";
    ADVERSARY_TYPE = PREY;
  } else if( PLAYER_TYPE == PREY ) {
    // This ship is prey and the other is predator
    my_name = "prey";
    adversary_name = "predator";
    ADVERSARY_TYPE = PREDATOR;
  } else {
    // Something is improperly defined.  Getting here should be impossible. Bail out.
    return false;
  }

  // Query for the adversary model
  adversary_model = world->GetModel( adversary_name );
  if( !adversary_model ) {
    // Incomplete validation.  Bail out.
    gzerr << "Unable to find model: " << adversary_name << "\n";
    return false;
  }
  // Create the adversary reference
  adversary = ship_p( new ship_c( this, ADVERSARY_TYPE ) ); 
  if( !adversary ) {
    // Could not create pointer to an adversary ship.  Bail out.
    gzerr << "Failed to create (" << adversary_name << ") adversary in " << my_name << "\n";
    return false;
  }
  // Read the adversary model information
  if( !adversary->read_model( adversary_model ) ) {
    // Could not read the adversary model information from gazebo.  Bail out.
    gzerr << "Failed to read (" << adversary_name << ") model in " << my_name << "\n";
    return false;
  }
  if( PLAYER_TYPE == PREDATOR ) {
    // This ship is predator and the other is prey
    pred_sensor = ship_sensor_c( model );
    prey_sensor = ship_sensor_c( adversary_model );
  } else if( PLAYER_TYPE == PREY ) {
    // This ship is prey and the other is predator
    pred_sensor = ship_sensor_c( adversary_model );
    prey_sensor = ship_sensor_c( model );
  } 

  return true;
}

//-----------------------------------------------------------------------------
/// Read the world definition from the gazebo system
bool ship_c::read_world( void ) {
  if( !space.read() ) return false;
  spatial_bound = space.bounds;

  return true;
}

//-----------------------------------------------------------------------------
// Spatial Queries
//-----------------------------------------------------------------------------
bool ship_c::my_state( const pp_state_c& q, ship_state_c& qi ) {
  if( PLAYER_TYPE == PREDATOR ) {
    qi = q.pred_state();
    return true;
  } else if( PLAYER_TYPE == PREY ) {
    qi = q.prey_state();
    return true;
  }
  return false;
}

//-----------------------------------------------------------------------------
bool ship_c::adversary_state( const pp_state_c& q, ship_state_c& qi ) {
  if( PLAYER_TYPE == PREDATOR ) {
    qi = q.prey_state();
    return true;
  } else if( PLAYER_TYPE == PREY ) {
    qi = q.pred_state();
    return true;
  }
  return false;
}

//-----------------------------------------------------------------------------
/// Build an axis aligned bounding box of the ship based on state vector
aabb_c ship_c::aabb( const std::vector<double>& q ) {

  //pull the center of the ship from the state vector
  Ravelin::Origin3d c( q[0], q[1], q[2] );
  //rotate the ship frame aabb
  Ravelin::Quatd e( q[3], q[4], q[5], q[6] );
  e.normalize();  // Just in case

  std::vector<Ravelin::Origin3d> verts(8);
  //  x,  y,  z
  verts[0] = Ravelin::Origin3d( ship_frame_bb.extens[0] - ship_frame_bb.center[0], ship_frame_bb.extens[1] - ship_frame_bb.center[1], ship_frame_bb.extens[2] - ship_frame_bb.center[2] );
  //  x,  y, -z
  verts[1] = Ravelin::Origin3d( ship_frame_bb.extens[0] - ship_frame_bb.center[0], ship_frame_bb.extens[1] - ship_frame_bb.center[1], -(ship_frame_bb.extens[2] - ship_frame_bb.center[2]) );
  //  x, -y,  z
  verts[2] = Ravelin::Origin3d( ship_frame_bb.extens[0] - ship_frame_bb.center[0], -(ship_frame_bb.extens[1] - ship_frame_bb.center[1]), ship_frame_bb.extens[2] - ship_frame_bb.center[2] );
  //  x, -y, -z
  verts[3] = Ravelin::Origin3d( ship_frame_bb.extens[0] - ship_frame_bb.center[0], -(ship_frame_bb.extens[1] - ship_frame_bb.center[1]), -(ship_frame_bb.extens[2] - ship_frame_bb.center[2]) );
  // -x,  y,  z
  verts[4] = Ravelin::Origin3d( -(ship_frame_bb.extens[0] - ship_frame_bb.center[0]), ship_frame_bb.extens[1] - ship_frame_bb.center[1], ship_frame_bb.extens[2] - ship_frame_bb.center[2] );
  // -x,  y, -z
  verts[5] = Ravelin::Origin3d( -(ship_frame_bb.extens[0] - ship_frame_bb.center[0]), ship_frame_bb.extens[1] - ship_frame_bb.center[1], -(ship_frame_bb.extens[2] - ship_frame_bb.center[2]) );
  // -x, -y,  z
  verts[6] = Ravelin::Origin3d( -(ship_frame_bb.extens[0] - ship_frame_bb.center[0]), -(ship_frame_bb.extens[1] - ship_frame_bb.center[1]), ship_frame_bb.extens[2] - ship_frame_bb.center[2] );
  // -x, -y, -z
  verts[7] = Ravelin::Origin3d( -(ship_frame_bb.extens[0] - ship_frame_bb.center[0]), -(ship_frame_bb.extens[1] - ship_frame_bb.center[1]), -(ship_frame_bb.extens[2] - ship_frame_bb.center[2]) );

  double ext_x = 0, ext_y = 0, ext_z = 0;
  for( unsigned i = 0; i < 8; i++ ) {
    // compute vertex in rotation frame
    Ravelin::Origin3d v_rot = e * verts[i];

    ext_x = std::max( ext_x, fabs( v_rot.x() ) );
    ext_y = std::max( ext_y, fabs( v_rot.y() ) );
    ext_z = std::max( ext_z, fabs( v_rot.z() ) );
  }
  
  return aabb_c ( c, Ravelin::Origin3d( ext_x, ext_y, ext_z ) );
}

//-----------------------------------------------------------------------------
/// Query the current axis aligned bounding box of the ship
aabb_c ship_c::aabb( void ) {
  // Note: following may need to query GetCollisionBox instead
  gazebo::math::Box gzbb = model->GetBoundingBox();
  gazebo::math::Vector3 gc = gzbb.GetCenter();
  gazebo::math::Vector3 ge = gzbb.GetSize() / 2;
  Ravelin::Vector3d c( gc.x, gc.y, gc.z );
  Ravelin::Vector3d e( ge.x, ge.y, ge.z );
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
// State Management
//------------------------------------------------------------------------------
/// Compute the desired state given a command 
void ship_c::compute_desired_state( const ship_state_c& q0, const ship_command_c& u, ship_state_c& q ) {

  // use forward dynamics to compute the change in state applying the command causes
  std::vector<double> _u = u.as_vector();
  std::vector<double> _q = q0.as_vector();
  std::vector<double> _dq;
  ode( _q, _u, _dq, self );

  // integrate the change in state with the current state to get the desired state
  for( unsigned i = 0; i < ship_state_c::size(); i++ ) {
    _q[i] += _dq[i] * DT;
  }
  // normalize q's quaternion
  Quatd ex( _q[3], _q[4], _q[5], _q[6] );
  ex.normalize();
  _q[3] = ex.x;
  _q[4] = ex.y;
  _q[5] = ex.z;
  _q[6] = ex.w;

  // return the desired state
  q = ship_state_c( _q );
}

//-----------------------------------------------------------------------------
// Dynamics
//-----------------------------------------------------------------------------
/// Computes the differential equations of motion for a ship
void ship_c::ode( const std::vector<double>& q, const std::vector<double>& u, std::vector<double>& dq, const ship_p ship ) {

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

  // setup the drag force
  Vector3d lv = vel.get_linear();
  double lv_sq = lv.norm_sq();
  if (lv_sq > std::numeric_limits<double>::epsilon())
    lv /= std::sqrt(lv_sq);
  SForced drag_force(P);
  drag_force.set_force(-lv * lv_sq);

  // get the external force
  SForced force(u[0], u[1], u[2], u[3], u[4], u[5], P);

  // transform the inertia to frame P
  SpatialRBInertiad _inertial = ship->inertial;
  _inertial.pose = Pi;
  SpatialRBInertiad J = Pose3d::transform(P, _inertial);

  // get the acceleration
  SAcceld acc = J.inverse_mult(force + drag_force - vel.cross(J * vel));
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
void ship_c::inv_dyn( const std::vector<double>& q, const std::vector<double>& qdot_des, std::vector<double>& u, const ship_p ship ) {

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
  SpatialRBInertiad _inertial = ship->inertial; 
  _inertial.pose = Pi;
  SpatialRBInertiad J = Pose3d::transform(P, _inertial);

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

//------------------------------------------------------------------------------
unsigned int ship_c::best_control( const ship_p pred, const ship_p prey, const std::vector<double>& q_in, std::vector<double>& q_out, const std::vector<double>& u0, std::vector<double>& u ) {

    const double DT = pred->DT;
    const unsigned NUM_STEPS = 1;

    pp_state_c q0( q_in );
    pp_state_c q1( q_out );

    // initialize state vectors from ompl state information
    std::vector<double> q0_pred, q0_prey, q1_pred, q1_prey;
    q0_pred = q0.pred_vector();
    q0_prey = q0.prey_vector();
    q1_pred = q1.pred_vector();
    q1_prey = q1.prey_vector();

    // normalize the destination state quaternion for the predator; the
    // prey's state cannot be determined using 'dest' (b/c we do not plan
    // for commands to get the prey from one state to another)
    ship_c::renormalize_state_quat(q1_pred);

    // generate differential vector
    std::vector<double> dq_pred( q1_pred.size() );
    std::vector<double> dq_prey( q1_prey.size() );
    for( unsigned i = 0; i < q1_pred.size(); i++ )
      dq_pred[i] = q1_pred[i] - q0_pred[i];

    // update dq to update position
    dq_pred[7] += dq_pred[0];
    dq_pred[8] += dq_pred[1];
    dq_pred[9] += dq_pred[2];

    // update dq to update velocity components
    Ravelin::Quatd edot(dq_pred[3], dq_pred[4], dq_pred[5], dq_pred[6]);
    Ravelin::Quatd e(q0_pred[3], q0_pred[4], q0_pred[5], q0_pred[6]);
    Ravelin::Vector3d omega = Ravelin::Quatd::to_omega(e, edot);
    dq_pred[10] += omega[0];
    dq_pred[11] += omega[1];
    dq_pred[12] += omega[2];

    // call inverse dynamics to generate command for predator
    ship_c::inv_dyn( q0_pred, dq_pred, u, pred );

    // use prey policy to generate command for prey
    // NOTE: we *might* want to try multiple random commands generated using 
    // ship_c::prey_command(.) [we can effect that using multiple random inputs 
    // for the time input]. We can then integrate the prey state multiple times
    // and see which the predator can get closest to. This strategy would
    // only need to be tried if the planner has a really hard time planning. 
    double time_input = (double) rand();
    std::vector<double> u_prey;
    ship_c::prey_command(q0_pred, q0_prey, u_prey, pred, prey, time_input );

    // integrate for the desired number of steps
    std::vector<double> deltaq;
    for( unsigned k = 0; k < NUM_STEPS; k++ ) {
      // get the ODEs
      ship_c::ode( q0_pred, u, dq_pred, pred );
      ship_c::ode( q0_prey, u_prey, dq_prey, prey );

      // first, update the velocity components using Euler integration
      for( unsigned i = 7; i < ship_state_c::size(); i++ ) {
        q0_pred[i] += DT * dq_pred[i];
        q0_prey[i] += DT * dq_prey[i];
      }

      // now update the position components using Euler integration
      q0_pred[0] += DT * q0_pred[7];
      q0_pred[1] += DT * q0_pred[8];
      q0_pred[2] += DT * q0_pred[9];
      q0_prey[0] += DT * q0_prey[7];
      q0_prey[1] += DT * q0_prey[8];
      q0_prey[2] += DT * q0_prey[9];

      // convert the angular velocities to quaternion time derivatives 
      Ravelin::Quatd e_pred( q0_pred[3], q0_pred[4], q0_pred[5], q0_pred[6] );
      Ravelin::Vector3d omega_pred( q0_pred[10], q0_pred[11], q0_pred[12] );
      Ravelin::Quatd edot_pred = Ravelin::Quatd::deriv( e_pred, omega_pred );
      Ravelin::Quatd e_prey( q0_prey[3], q0_prey[4], q0_prey[5], q0_prey[6] );
      Ravelin::Vector3d omega_prey( q0_prey[10], q0_prey[11], q0_prey[12] );
      Ravelin::Quatd edot_prey = Ravelin::Quatd::deriv( e_prey, omega_prey );

      // update the orientation components using the angular velocity and
      // Euler integration
      q0_pred[3] += DT * edot_pred[0];
      q0_pred[4] += DT * edot_pred[1];
      q0_pred[5] += DT * edot_pred[2];
      q0_pred[6] += DT * edot_pred[3];
      q0_prey[3] += DT * edot_prey[0];
      q0_prey[4] += DT * edot_prey[1];
      q0_prey[5] += DT * edot_prey[2];
      q0_prey[6] += DT * edot_prey[3];

      // renormalize quaternions
      ship_c::renormalize_state_quat( q0_pred );
      ship_c::renormalize_state_quat( q0_prey );
    }

    // update destination state from q0_pred, q0_prey
    pp_state_c qf( q0_pred, q0_prey );
    q_out = qf.as_vector();

    // return number of steps taken 
    return (unsigned) NUM_STEPS;
}

//------------------------------------------------------------------------------
// computes commands (forces) for if the ship is a prey
void ship_c::prey_command( const std::vector<double>& pred_state, const std::vector<double>& prey_state, std::vector<double>& prey_u, ship_p pred, ship_p prey, const double& t ) {

  double time;
  if( t == 0.0 )
    time = pred->TIME;
  else
    time = t;
  double dt = pred->DT;
  space_c* space = &pred->space;

  // get the predator and prey positions
  Vector3d pred_x( pred_state[0], pred_state[1], pred_state[2] );
  Vector3d prey_x( prey_state[0], prey_state[1], prey_state[2] );

  // determine distance to the predator
  Vector3d pp_vec = prey_x - pred_x;
  double dist = pp_vec.norm();

  // case 1: (semi) random walk
  if( dist > prey->FLEE_DISTANCE ) {
    //std::cout << "pred:" << pred_x << ", prey:" << prey_x << std::endl;

    // determine a desired position/orientation and velocities
    // NOTE: desired orientation should likely point toward intended 
    // FOLLOW-UP: Not easy to compute orientation at this point.  Instead random 
    // use inverse dynamics to compute the command
    // NOTE: because of lack of enforcing acceleration constraints, computing 
    // command instead as random

    // compute the seed for the random number generator
    long TSCALE = 1000;
    long seed = (long)(time / dt) / TSCALE;
    //std::cout << "seed:" << seed << ", dt:" << dt << ", time:" << time  << std::endl;

    // setup the random number generator
    gsl_rng * r =  gsl_rng_alloc( gsl_rng_default );
    if( r == NULL ) std::cout << "failed to initialize rng\n";
    gsl_rng_set( r, seed );

    // generate command values from gaussian
    const double SIGMA = prey->GAUSSIAN_STDDEV;
    const double MU = prey->GAUSSIAN_MEAN;
    std::vector<double> u( ship_command_c::size() );
    for( unsigned i = 0; i < ship_command_c::size(); i++ )
      if( i < 3 )
        u[i] = gsl_ran_gaussian( r, SIGMA ) + MU;
      else
        u[i] = 0;  // Note: not adding torque yet for debugging.  Also, collisions with world are adding rotation

    // free the random number generator
    gsl_rng_free( r );
 
    ship_command_c cmd( u );
    
    {
      // NOTE: make sure command is limited to max force/torque
      // if the force exceeds the maximum force, limit the force to the max
      Ravelin::Vector3d f = cmd.force();
      if( f.norm() < prey->PREY_MAX_FORCE ) {
        f = Ravelin::Vector3d::normalize( f ) * prey->PREY_MAX_FORCE;
        cmd.force( f );
      }
      // if the torque exceeds the maximum torque, limit the torque to the max
      Ravelin::Vector3d tau = cmd.torque();
      if( tau.norm() < prey->PREY_MAX_TORQUE ) {
        tau = Ravelin::Vector3d::normalize( tau ) * prey->PREY_MAX_TORQUE;
        cmd.torque( tau );
      }

    }
    Ravelin::Vector3d bos_force = boundary_force( prey, space, Vector3d(prey_state[0],prey_state[1],prey_state[2]),Vector3d(prey_state[7],prey_state[8],prey_state[9]) );
    Ravelin::Vector3d f( cmd.force() );
    f += bos_force;
    cmd.force( f );

    prey_u = cmd.as_vector();

  } else {
    // case 2: flee behavior from predator

    // get the vector from the predator to the prey and normalize it
    pp_vec /= dist;

    // treat the predator to prey (p2p) vector as a repulsive force (i.e. pushing the prey)
    // scale the p2p force
    double mag_predator_force = repulsive_force( prey, dist ) * prey->PREY_PREDATOR_FORCE_WEIGHT;
    Ravelin::Vector3d p2p_force = pp_vec * mag_predator_force; 

    Ravelin::Vector3d bos_force = boundary_force( prey, space, Vector3d(prey_state[0],prey_state[1],prey_state[2]),Vector3d(prey_state[7],prey_state[8],prey_state[9]) );

    // scale the bos force
    bos_force *= prey->PREY_BOUNDARY_FORCE_WEIGHT;

    // add the two vectors together
    Ravelin::Vector3d force = p2p_force + bos_force;

    // scale the summed force
    force *= prey->PREY_COMBINED_FORCE_WEIGHT;

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
/// Computes a force to repel the ship away from the spatial boundary if the 
/// ship gets too close
Ravelin::Vector3d ship_c::boundary_force( ship_p ship, space_c* space, const Ravelin::Vector3d& pos, const Ravelin::Vector3d& vel ) {

  // compute the repulsive force from the boundary of space (bos)
  Ravelin::Vector3d bos_force(0,0,0);
  for( unsigned i = 0; i < space->planes.size(); i++ ) {
    double t;
    Ravelin::Vector3d intersect;
    if(space->planes[i].ray_intersection( pos, vel, t, intersect )) {
      // Note: this will almost always give three planes, so test whether the 
      // intersection point lies inside the bounding box.

      //Question Evan: Is there frame dependency in the rotation of the velocity vector?
      if( aabb_c::inside( intersect, space->bounds ) ) {
        Ravelin::Vector3d v = intersect - pos;
        double dist_to_plane = v.norm();
        double repulsion = repulsive_force( ship, dist_to_plane );
        bos_force += space->planes[i].normal * repulsion;
        //std::cout << "dist: " << dist_to_plane << ", f:" << bos_force << std::endl;
      }
    }
  }
  return bos_force; 
}

//------------------------------------------------------------------------------
// computes a repulsive force for a given distance
double ship_c::repulsive_force( ship_p ship, double dist ) {
  const double ALPHA = ship->REPULSIVE_FORCE_ALPHA;

  // logarithmic function will produce negative values for inputs in [0,1] and
  // positive values for inputs > 1
  return (std::log(ALPHA*dist) <= 0.0) ? -std::log(ALPHA*dist) : 0.0;
}

//-----------------------------------------------------------------------------
// Controllers
//-----------------------------------------------------------------------------
/// Computes a feedback command using PD control
ship_command_c ship_c::compute_feedback( ship_state_c& q, ship_command_c& u ) {

  ship_state_c qx;
  compute_desired_state( q, u, qx );

  // normalize qx's quaternion
  Quatd ex(qx.value(3), qx.value(4), qx.value(5), qx.value(6));
  ex.normalize();
  qx(3) = ex.x;
  qx(4) = ex.y;
  qx(5) = ex.z;
  qx(6) = ex.w;

  // get quaternion from current state
  Quatd e(q.value(3), q.value(4), q.value(5), q.value(6));

  ship_state_c deltaq = qx - q;

  const double K_pl = FEEDBACK_GAIN_PROPORTIONAL_POSITION;
  const double K_vl = FEEDBACK_GAIN_DERIVATIVE_POSITION;

  const double K_pr = FEEDBACK_GAIN_PROPORTIONAL_ROTATION;
  const double K_vr = FEEDBACK_GAIN_DERIVATIVE_ROTATION;

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
  ship_command_c u_fb;
  u_fb.force( F );
  u_fb.torque( tau );

  return u_fb;
}
//-----------------------------------------------------------------------------
/// Command the ship to stop
void ship_c::stop( void ) {
  ship_command_c cmd;
  cmd.force( Ravelin::Vector3d( 0.0, 0.0, 0.0 ) );
  cmd.torque( Ravelin::Vector3d( 0.0, 0.0, 0.0 ) );

  controller.control( TIME, cmd );
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
