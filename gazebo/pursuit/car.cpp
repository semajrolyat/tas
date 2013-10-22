#include <car.h>

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
GZ_REGISTER_MODEL_PLUGIN( car_c )

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
car_c::car_c( void ) { 

}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
car_c::~car_c( void ) {
    gazebo::event::Events::DisconnectWorldUpdateBegin( this->updateConnection );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void car_c::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model = _model;
  world = _model->GetWorld();

  //---------------------------------------------------------------------------
  // Links
  //---------------------------------------------------------------------------
  body = model->GetLink("body");
  if( !body ) {
    gzerr << "Unable to find link: body\n";
    return;
  }

  rear_driveshaft = model->GetLink("rear_driveshaft");
  if( !rear_driveshaft ) {
    gzerr << "Unable to find link: rear_driveshaft\n";
    return;
  }

  steering_link = model->GetLink("steering_link");
  if( !steering_link ) {
    gzerr << "Unable to find link: steering_link\n";
    return;
  }

  spindle_front_left = model->GetLink("spindle_front_left");
  if( !spindle_front_left ) {
    gzerr << "Unable to find link: spindle_front_left\n";
    return;
  }

  spindle_front_right = model->GetLink("spindle_front_right");
  if( !spindle_front_right ) {
    gzerr << "Unable to find link: spindle_front_right\n";
    return;
  }

  wheel_front_left = model->GetLink("wheel_front_left");
  if( !wheel_front_left ) {
    gzerr << "Unable to find link: wheel_front_left\n";
    return;
  }

  wheel_front_right = model->GetLink("wheel_front_right");
  if( !wheel_front_right ) {
    gzerr << "Unable to find link: wheel_front_right\n";
    return;
  }

  wheel_rear_left = model->GetLink("wheel_rear_left");
  if( !wheel_rear_left ) {
    gzerr << "Unable to find link: wheel_rear_left\n";
    return;
  }

  wheel_rear_right = model->GetLink("wheel_rear_right");
  if( !wheel_rear_right ) {
    gzerr << "Unable to find link: wheel_rear_right\n";
    return;
  }

  //---------------------------------------------------------------------------
  // Joints
  //---------------------------------------------------------------------------
  steering_box = model->GetJoint("steering_box");
  if( !steering_box ) {
    gzerr << "Unable to find joint: steering_box\n";
    return;
  }

  rear_differential = model->GetJoint("rear_differential");
  if( !rear_differential ) {
    gzerr << "Unable to find joint: rear_differential\n";
    return;
  }

  kingpin_front_left = model->GetJoint("kingpin_front_left");
  if( !kingpin_front_left ) {
    gzerr << "Unable to find joint: kingpin_front_left\n";
    return;
  }

  kingpin_front_right = model->GetJoint("kingpin_front_right");
  if( !kingpin_front_right ) {
    gzerr << "Unable to find joint: kingpin_front_right\n";
    return;
  }

  bearing_front_left = model->GetJoint("bearing_front_left");
  if( !bearing_front_left ) {
    gzerr << "Unable to find joint: bearing_front_left\n";
    return;
  }

  bearing_front_right = model->GetJoint("bearing_front_right");
  if( !bearing_front_right ) {
    gzerr << "Unable to find joint: bearing_front_right\n";
    return;
  }

  bearing_rear_left = model->GetJoint("bearing_rear_left");
  if( !bearing_rear_left ) {
    gzerr << "Unable to find joint: bearing_rear_left\n";
    return;
  }

  bearing_rear_right = model->GetJoint("bearing_rear_right");
  if( !bearing_rear_right ) {
    gzerr << "Unable to find joint: bearing_rear_right\n";
    return;
  }

  //---------------------------------------------------------------------------
  // Parameters
  //---------------------------------------------------------------------------
  if( !_sdf->HasElement("max_speed") ) {
    gzerr << "Unable to find parameter: max_speed\n";
    return;
  } 
  MAX_SPEED = _sdf->GetElement( "max_speed" )->GetValueDouble();

  if( !_sdf->HasElement("max_steering_velocity") ) {
    gzerr << "Unable to find parameter: max_steering_velocity\n";
    return;
  } 
  MAX_STEERING_VELOCITY = _sdf->GetElement( "max_steering_velocity" )->GetValueDouble();

  if( !_sdf->HasElement("steering_control_kp") ) {
    gzerr << "Unable to find parameter: steering_control_kp\n";
    return;
  } 
  STEERING_CONTROL_KP = _sdf->GetElement( "steering_control_kp" )->GetValueDouble();

  if( !_sdf->HasElement("steering_control_kd") ) {
    gzerr << "Unable to find parameter: steering_control_kd\n";
    return;
  } 
  STEERING_CONTROL_KD = _sdf->GetElement( "steering_control_kd" )->GetValueDouble();

  if( !_sdf->HasElement("speed_control_kp") ) {
    gzerr << "Unable to find parameter: speed_control_kp\n";
    return;
  } 
  SPEED_CONTROL_KP = _sdf->GetElement( "speed_control_kp" )->GetValueDouble();

  if( !_sdf->HasElement("max_steering_angle_deg") ) {
    gzerr << "Unable to find parameter: max_steering_angle_deg\n";
    return;
  } 
  MAX_STEERING_ANGLE = _sdf->GetElement( "max_steering_angle_deg" )->GetValueDouble() * PI / 180.0;

  if( !_sdf->HasElement("steering_lever_base_angle_deg") ) {
    gzerr << "Unable to find parameter: steering_lever_base_angle_deg\n";
    return;
  } 
  STEERING_LEVER_BASE_ANGLE = _sdf->GetElement( "steering_lever_base_angle_deg" )->GetValueDouble() * PI / 180.0;

  if( !_sdf->HasElement("kingpin_distance") ) {
    gzerr << "Unable to find parameter: kingpin_distance\n";
    return;
  } 
  BASE_LINK_LENGTH = _sdf->GetElement( "kingpin_distance" )->GetValueDouble();

  if( !_sdf->HasElement("steering_lever_length") ) {
    gzerr << "Unable to find parameter: steering_lever_length\n";
    return;
  } 
  STEERING_LEVER_LENGTH = _sdf->GetElement( "steering_lever_length" )->GetValueDouble();

  if( !_sdf->HasElement("tie_rod_length") ) {
    gzerr << "Unable to find parameter: tie_rod_length\n";
    return;
  } 
  TIEROD_LENGTH = _sdf->GetElement( "tie_rod_length" )->GetValueDouble();

  if( !_sdf->HasElement("spindle_lever_length") ) {
    gzerr << "Unable to find parameter: spindle_lever_length\n";
    return;
  } 
  SPINDLE_LEVER_LENGTH = _sdf->GetElement( "spindle_lever_length" )->GetValueDouble();

  // Compute Steering Linkage Helpers
  BASE_LINK_LENGTH_SQ = BASE_LINK_LENGTH * BASE_LINK_LENGTH;
  STEERING_LEVER_LENGTH_SQ = STEERING_LEVER_LENGTH * STEERING_LEVER_LENGTH;
  TIEROD_LENGTH_SQ = TIEROD_LENGTH * TIEROD_LENGTH;
  SPINDLE_LEVER_LENGTH_SQ = SPINDLE_LEVER_LENGTH * SPINDLE_LEVER_LENGTH;
 
  WHEEL_BASE = (steering_link->GetWorldPose().pos - rear_driveshaft->GetWorldPose().pos).GetLength();

  // Derive Steering Linkage Variables
  Real theta = STEERING_LEVER_BASE_ANGLE;
  Real h_sq = BASE_LINK_LENGTH_SQ + STEERING_LEVER_LENGTH_SQ - 
          2.0 * BASE_LINK_LENGTH * STEERING_LEVER_LENGTH * cos( theta );
  Real h = sqrt( h_sq );
  Real rho = acos( (BASE_LINK_LENGTH_SQ + h_sq - STEERING_LEVER_LENGTH_SQ) / 
          (2.0 * BASE_LINK_LENGTH * h) ) +
          acos( (SPINDLE_LEVER_LENGTH_SQ + h_sq - TIEROD_LENGTH_SQ) / 
          (2.0 * SPINDLE_LEVER_LENGTH * h) );
  SPINDLE_LEVER_BASE_ANGLE = PI - rho;


  //---------------------------------------------------------------------------
  // Testing Parameters
  //---------------------------------------------------------------------------
  if( !_sdf->HasElement("lissajous_ks") ) {
    gzerr << "Unable to find parameter: lissajous_ks\n";
    return;
  } 
  LISSAJOUS_KS = _sdf->GetElement( "lissajous_ks" )->GetValueDouble();

  if( !_sdf->HasElement("lissajous_kd") ) {
    gzerr << "Unable to find parameter: lissajous_kd\n";
    return;
  } 
  LISSAJOUS_KD = _sdf->GetElement( "lissajous_kd" )->GetValueDouble();

  if( !_sdf->HasElement("lissajous_du_speed") ) {
    gzerr << "Unable to find parameter: lissajous_du_speed\n";
    return;
  } 
  LISSAJOUS_DU_SPEED = _sdf->GetElement( "lissajous_du_speed" )->GetValueDouble();

  if( !_sdf->HasElement("lissajous_du_angle") ) {
    gzerr << "Unable to find parameter: lissajous_du_angle\n";
    return;
  } 
  LISSAJOUS_DU_ANGLE = _sdf->GetElement( "lissajous_du_angle" )->GetValueDouble();

  //---------------------------------------------------------------------------
  // Call Init Function (Note: target function subject to inheritance)
  //---------------------------------------------------------------------------
  init();
}

//-----------------------------------------------------------------------------
void car_c::Update( ) {

  //compensate_for_roll_pitch( );

  time = world->GetSimTime().Double() - time_start;
  dtime = time - time_last;

  if( states.size() == 0 )
    plan_lissajous();

  sense( );
  plan( );
  act( );

  //test_state_functions();
  //test_command_functions();

  time_last = time;
}

//-----------------------------------------------------------------------------
/*
void car_c::Reset( ) {

}
*/

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void car_c::init( void ) {
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind( &car_c::Update, this ) );

  audit_file_planned_commands = "car_planned_commands.txt";
  audit_file_planned_states = "car_planned_states.txt";
  audit_file_actual_commands = "car_actual_commands.txt";
  audit_file_actual_states = "car_actual_states.txt";
  write_command_audit_header( audit_file_planned_commands );
  write_state_audit_header( audit_file_planned_states );
  write_command_audit_header( audit_file_actual_commands );
  write_state_audit_header( audit_file_actual_states );

  std::cerr << "Car Controller Plugin Started" << std::endl;

  time_start = world->GetSimTime().Double();
  time_last = time_start;
}

//-----------------------------------------------------------------------------
void car_c::sense( void ) {

}

//-----------------------------------------------------------------------------
void car_c::plan( void ) {

}

//-----------------------------------------------------------------------------
void car_c::act( void ) {

  if( active_command.duration <= 0.0 ) {
    if( !commands.size() ) return;

    active_command = commands.front();
    commands.erase( commands.begin() );
  }
  
  active_command.duration -= dtime;
///*
  steer( active_command );
  push( active_command );
//*/

  car_command_c actual_command( dtime, speed(), steering_angle() );
  car_state_c actual_state = state();
  write_audit_datum( audit_file_actual_commands, actual_command );
  write_audit_datum( audit_file_actual_states, actual_state );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
car_state_c car_c::state( void ) {
  gazebo::math::Vector3 pos = rear_driveshaft->GetWorldPose().pos;
  gazebo::math::Quaternion rot = model->GetWorldPose().rot;
  Real x = pos.x;
  Real y = pos.y;
  Real theta = rot.GetYaw();
  return car_state_c( x, y, theta );
}

//-----------------------------------------------------------------------------
void car_c::state( const car_state_c& _state ) {
  gazebo::math::Quaternion r = gazebo::math::Quaternion(0.0, 0.0, _state.theta);
  gazebo::math::Vector3 coa = gazebo::math::Vector3( _state.x, _state.y, 0.0 );
  gazebo::math::Vector3 d = -rear_driveshaft->GetRelativePose().pos;
  d.z = 0.0;
  gazebo::math::Vector3 com = r.RotateVector( d ) + coa;
  model->SetWorldPose( gazebo::math::Pose( com, r ) );
}

//-----------------------------------------------------------------------------
gazebo::math::Vector3 car_c::orientation( void ) {
  return (steering_link->GetWorldPose().pos - rear_driveshaft->GetWorldPose().pos).Normalize();
}

//-----------------------------------------------------------------------------
Real car_c::steering_angle( void ) {
  return steering_box->GetAngle( 0 ).Radian();
}

//-----------------------------------------------------------------------------
Real car_c::speed( void ) {
  return model->GetRelativeLinearVel().x;
}

//-----------------------------------------------------------------------------
void car_c::steer( const car_command_c& command ) {

  //steer_direct( 0.0 );
  steer_direct( steering_angle() );
  //steer_double_four_bar( steering_angle() );

  Real desired_angle = command.angle;
  Real actual_angle = steering_angle();
  //Real desired_velocity = 0.0;
  Real desired_velocity = MAX_STEERING_VELOCITY;
  Real actual_velocity = steering_box->GetVelocity( 0 );
///*
  Real torque = STEERING_CONTROL_KP * ( desired_angle - actual_angle ) + 
                STEERING_CONTROL_KD * ( desired_velocity - actual_velocity );
//*/
/*
  Real torque = STEERING_CONTROL_KP * ( desired_angle - actual_angle ) + 
                STEERING_CONTROL_KD * ( 0.0 - actual_velocity );
*/
  steering_box->SetForce( 0, torque );

}

//-----------------------------------------------------------------------------
void car_c::push( const car_command_c& command ) {

  Real desired_speed = command.speed;
  Real actual_speed = speed();
///*
  Real f = SPEED_CONTROL_KP * ( desired_speed - actual_speed );
//*/
/*
  Real f = SPEED_CONTROL_KP * ( desired_speed - actual_speed ) +
           50.0 * ( 2.0 - model->GetRelativeLinearAccel().x);
*/
  /*
  car_state_c _state = state();
  gazebo::math::Quaternion r = gazebo::math::Quaternion(0.0, 0.0, _state.theta);
  gazebo::math::Vector3 _speed = gazebo::math::Vector3( f,0.0,0.0 );
  gazebo::math::Vector3 v = r.RotateVector( _speed );
  body->AddForce( v );
  */
/*
  car_state_c _state = state();
  gazebo::math::Quaternion r = gazebo::math::Quaternion(0.0, 0.0, _state.theta);
  gazebo::math::Vector3 _x = gazebo::math::Vector3( 1.0, 0.0, 0.0 );
  gazebo::math::Vector3 d = r.RotateVector( _x );
  d.z = 0.0;
  gazebo::math::Vector3 v = d.Normalize() * f;
  body->AddForce( v );
*/


  gazebo::math::Vector3 d = orientation( );
  gazebo::math::Vector3 v = d * f;
  body->AddForce( v );

} 

//-----------------------------------------------------------------------------
void car_c::steer_direct( const Real& _steering_angle ) {

  kingpin_front_left->SetAngle( 0, gazebo::math::Angle( _steering_angle ) );
  kingpin_front_right->SetAngle( 0, gazebo::math::Angle( _steering_angle ) );

/*
  gazebo::math::Pose l = wheel_front_left->GetRelativePose();
  gazebo::math::Pose r = wheel_front_right->GetRelativePose();

  l.Set( l.pos.x, l.pos.y, l.pos.z, l.rot.GetRoll(), l.rot.GetPitch(), _steering_angle );
  r.Set( r.pos.x, r.pos.y, r.pos.z, r.rot.GetRoll(), r.rot.GetPitch(), _steering_angle );

  wheel_front_left->SetRelativePose( l );
  wheel_front_right->SetRelativePose( r );
*/
}

//-----------------------------------------------------------------------------
void car_c::steer_double_four_bar( const Real& _steering_angle ) {

  Real right_angle =  double_four_bar_kingpin_angle( _steering_angle );
  Real left_angle = -double_four_bar_kingpin_angle( -_steering_angle );

  kingpin_front_left->SetAngle( 0, gazebo::math::Angle( left_angle ) );
  kingpin_front_right->SetAngle( 0, gazebo::math::Angle( right_angle ) );

}

//-----------------------------------------------------------------------------
Real car_c::double_four_bar_kingpin_angle( const Real& _steering_angle ) {
  Real theta = STEERING_LEVER_BASE_ANGLE - _steering_angle;
  Real h_sq = BASE_LINK_LENGTH_SQ + STEERING_LEVER_LENGTH_SQ - 
          2.0 * BASE_LINK_LENGTH * STEERING_LEVER_LENGTH * cos( theta );
  Real h = sqrt( h_sq );

  Real rho = acos( (BASE_LINK_LENGTH_SQ + h_sq - STEERING_LEVER_LENGTH_SQ) / 
          (2.0 * BASE_LINK_LENGTH * h) ) +
          acos( (SPINDLE_LEVER_LENGTH_SQ + h_sq - TIEROD_LENGTH_SQ) / 
          (2.0 * SPINDLE_LEVER_LENGTH * h) );
  return rho - (PI - SPINDLE_LEVER_BASE_ANGLE);
}

//-----------------------------------------------------------------------------
car_state_c car_c::ode( const Real& theta, const car_command_c& command ) {
  Real dx = command.speed * cos( theta );
  Real dy = command.speed * sin( theta );
  Real dtheta = command.speed * tan( command.angle ) / WHEEL_BASE;

  return car_state_c( dx, dy, dtheta );
}


//------------------------------------------------------------------------------
Real car_c::speed_optimization_function( const Real& u_s, const Real& dx, const Real& dy, const Real& costheta, const Real& sintheta ) {
  return u_s * u_s + dx * dx + dy * dy - 2.0 * u_s * ( dx * costheta + dy * sintheta );
}

//------------------------------------------------------------------------------
car_command_c car_c::inverse_ode(const Real& theta, const car_state_c& dstate) {

  Real u_s, u_phi;

  // *First Compute Speed*
  // Attempt optimization of u_s using backtracking line search
  // Note: optimization is necessary because the reality is that dx/cos rarely 
  // equals dy/sin and by switching between sin and cos which must be done else 
  // DIV0, discontinuities are introduced that can be substantial changes.
  Real c = cos( theta );
  Real s = sin( theta );

  if( fabs(c) > 0.5 )
    u_s = dstate.x / c;
  else
    u_s = dstate.y / s;
  
  Real du, f0, f, d1, t;
  const Real alpha = 0.01;
  const Real beta = 0.5;
  int its = 0;
  const int MAX_ITS = 1000;
  const Real EPSILON = 1e-3;
  const Real EPSILON2 = 1e-3; 

  do{
    t = 1.0;

    d1 = 2.0 * u_s - 2.0 * ( dstate.x * c + dstate.y * s );
    du = -d1/2.0;
    f0 = speed_optimization_function(u_s, dstate.x, dstate.y, c, s );
    f = speed_optimization_function( u_s + t * du, dstate.x, dstate.y, c, s );
    while( f > f0 + alpha * t * d1 * du ) {
      t = beta * t;
      f = speed_optimization_function( u_s + t * du, dstate.x, dstate.y, c, s );
    }
    u_s = u_s + t * du;
  
  } while(its++ < MAX_ITS && f0 > EPSILON && fabs(du) > EPSILON2 );

  // *Second Compute Steering Angle*
  // atan2 ill behaved for (0,0) so handle that case if it arises *brute force*
  const Real EPSILON3 = 1e-3;
  if( fabs(dstate.theta) < EPSILON3 && fabs(u_s) < EPSILON3 )
    u_phi = 0.0;
  else
    // otherwise, use the formulation
    u_phi = atan2( dstate.theta * WHEEL_BASE, u_s );

  return car_command_c( u_s, u_phi );
}

//------------------------------------------------------------------------------

void car_c::compensate_for_roll_pitch( void ) {
  // For tight turns, this ensures that the model sticks to the ground
  // and doesn't roll over.  It can lead to skidding
  gazebo::math::Quaternion rot = model->GetWorldPose().rot;
  Real _roll = rot.GetRoll();
  Real _pitch = rot.GetPitch();
  Real _yaw = rot.GetYaw();
  gazebo::math::Vector3 pos = model->GetWorldPose().pos;
  gazebo::math::Quaternion rot_fixed( 0.0, 0.0, _yaw );
  model->SetWorldPose( gazebo::math::Pose( pos, rot_fixed ) );
}

//------------------------------------------------------------------------------
// Auditing 
//------------------------------------------------------------------------------
bool car_c::write_command_audit_header( const std::string& filename ) {
  std::fstream fs;
  fs.open( filename.c_str(), std::fstream::out );
  if( !fs.is_open() ) return false;
  fs << car_command_c::header() << std::endl;
  fs.close();
  return true;
}

//------------------------------------------------------------------------------
bool car_c::write_state_audit_header( const std::string& filename ) {
  std::fstream fs;
  fs.open( filename.c_str(), std::fstream::out );
  if( !fs.is_open() ) return false;
  fs << car_state_c::header() << std::endl;
  fs.close();
  return true;
}

//------------------------------------------------------------------------------
bool car_c::write_audit_datum( const std::string& filename, const car_command_c& cmd ) {
  std::fstream fs;
  fs.open( filename.c_str(), std::fstream::out | std::fstream::app );
  if( !fs.is_open() ) return false;
  fs << cmd << std::endl;
  fs.close();
  return true;
}

//------------------------------------------------------------------------------
bool car_c::write_audit_datum( const std::string& filename, const car_state_c& state ) {
  std::fstream fs;
  fs.open( filename.c_str(), std::fstream::out | std::fstream::app );
  if( !fs.is_open() ) return false;
  fs << state << std::endl;
  fs.close();
  return true;
}

//------------------------------------------------------------------------------
// Testing
//------------------------------------------------------------------------------
void car_c::test_state_functions( void ) {
  //Validate Car State Functions
  car_state_c _state_in = car_state_c( 0.0, 0.0, 0.0 );
  //car_state_c _state_in = car_state_c( 1.0, 1.0, 1.5708 );
  //car_state_c _state_in = car_state_c( 1.0, 1.0, 3.1416 );
  //car_state_c _state_in = car_state_c( 1.0, 1.0, -1.5708 );
  state( _state_in );

  car_state_c _state_out = state();
  std::cout << _state_out << std::endl;
}

//------------------------------------------------------------------------------
void car_c::test_command_functions( void ) {
  //car_command_c command = car_command_c( 50.0, 0.5236 );
  //car_command_c command = car_command_c( 1.0, 0.2 );
  //car_command_c command = car_command_c( 1.0, 0.2 );
  car_command_c command = car_command_c( 0.2, -0.5236 );

  steer( command );
  push( command );
}

//-----------------------------------------------------------------------------
// Planning
//-----------------------------------------------------------------------------
Real car_c::signed_angle( const Real& ux, const Real& uy, const Real& vx, const Real& vy ) {
  gazebo::math::Vector3 u( ux, uy, 0.0 );
  gazebo::math::Vector3 v( vx, vy, 0.0 );

  u = u.Normalize();
  v = v.Normalize();

  return atan2( u.x * v.y - u.y * v.z, u.Dot(v) );
}

//-----------------------------------------------------------------------------
car_state_c car_c::lissajous( const Real& t, const Real& a, const Real& b, const Real& A, const Real& B, const Real& delta ) { 
  // Coordinates at t
  Real x = A * sin( a * t + delta );
  Real y = B * sin( b * t );

  // Derivative at t
  Real dx = A * cos( a * t + delta ) * a; // by chain rule
  Real dy = B * cos( b * t ) * b;         // by chain rule

  // Angle in world frame at t
  Real theta = signed_angle( 1.0, 0.0, dx, dy );

  return car_state_c( x, y, theta );
}

//-----------------------------------------------------------------------------
void car_c::plan_lissajous( void ) {
  Real dt = 0.025;  // Note: this is the highest possible step size before loss of stability
  Real Ks = LISSAJOUS_KS;
  Real Kd = LISSAJOUS_KD;
  Real DU_speed = LISSAJOUS_DU_SPEED;
  Real DU_angle = LISSAJOUS_DU_ANGLE;
  Real period = 2 * PI;
  car_state_c initial( 2.0, 0.0, PI/2.0 );
  state( initial );
  car_state_c dq( 0.0, 0.0, 0.0 );
  car_state_c q = state();
  states.push_back( q );
  
  write_audit_datum( audit_file_planned_states, q );

  Real alpha = 10.0;
  for( Real t = dt; t <= period * alpha + dt; t += dt ) {
    car_state_c q_desired = lissajous( t / alpha, 1.0, 2.0, 2.0, 2.0, PI/2.0 );
    q = state();
    car_state_c dq_desired = q_desired - q;
    car_state_c delta_q = Ks * (q_desired - q) + Kd * (dq_desired - dq);
    car_command_c u = inverse_ode( q.theta, delta_q );

    u.speed *= DU_speed;
    u.angle *= DU_angle;
    u.duration = dt ;

    dq = ode( q.theta, u );

    q = q + dq * dt;

    state( q );

    //states.push_back( q_desired );
    //write_audit_datum( audit_file_planned_states, q_desired );
    commands.push_back( u );
    write_audit_datum( audit_file_planned_commands, u );
    states.push_back( q );
    write_audit_datum( audit_file_planned_states, q );
    //write_audit_datum( audit_file_planned_states, delta_q );
  }
  state( initial );
}

//-----------------------------------------------------------------------------

