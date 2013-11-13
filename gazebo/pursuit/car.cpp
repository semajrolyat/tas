#include <car.h>

//-----------------------------------------------------------------------------
Real sgn(Real x)
{
  if (x > 0.0)
    return 1.0;
  else if (x < 0.0)
    return -1.0;
  else
    return 0.0;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
GZ_REGISTER_MODEL_PLUGIN( car_c )

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
car_c::car_c( void ) { 

}
//-----------------------------------------------------------------------------
car_c::car_c( const ompl::base::StateSpace *space ) : space_(space) {

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

  if( !_sdf->HasElement("max_acceleration") ) {
    gzerr << "Unable to find parameter: max_acceleration\n";
    return;
  } 
  MAX_ACCELERATION = _sdf->GetElement( "max_acceleration" )->GetValueDouble();

  if( !_sdf->HasElement("max_steering_velocity") ) {
    gzerr << "Unable to find parameter: max_steering_velocity\n";
    return;
  } 
  MAX_STEERING_VELOCITY = _sdf->GetElement( "max_steering_velocity" )->GetValueDouble();

  // Planner Parameters
  if( !_sdf->HasElement("planner") ) {
    gzerr << "Unable to find parameter: planner\n";
    return;
  } 
  std::string planner = _sdf->GetElement( "planner" )->GetValueString();
  if( planner.compare("rrt") == 0 ) {
    PLANNER = RRT;
  } else if( planner.compare("lissajous") == 0 ) {
    PLANNER = LISSAJOUS;
  } else {
    PLANNER = LISSAJOUS;
  }

  if( PLANNER == RRT ) {
    if( !_sdf->HasElement("rrt_planner_step_size") ) {
      gzerr << "Unable to find parameter: rrt_planner_step_size\n";
      return;
    } 
    PLANNER_STEP_SIZE = _sdf->GetElement( "rrt_planner_step_size" )->GetValueDouble();
    if( !_sdf->HasElement("rrt_steering_control_kp") ) {
      gzerr << "Unable to find parameter: rrt_steering_control_kp\n";
      return;
    } 
    STEERING_CONTROL_KP = _sdf->GetElement( "rrt_steering_control_kp" )->GetValueDouble();

    if( !_sdf->HasElement("rrt_steering_control_kd") ) {
      gzerr << "Unable to find parameter: rrt_steering_control_kd\n";
      return;
    } 
    STEERING_CONTROL_KD = _sdf->GetElement( "rrt_steering_control_kd" )->GetValueDouble();

    if( !_sdf->HasElement("rrt_speed_control_kp") ) {
      gzerr << "Unable to find parameter: rrt_speed_control_kp\n";
      return;
    } 
    SPEED_CONTROL_KP = _sdf->GetElement( "rrt_speed_control_kp" )->GetValueDouble();

    if( !_sdf->HasElement("rrt_fb_ks") ) {
      gzerr << "Unable to find parameter: rrt_fb_ks\n";
      return;
    } 
    FEEDBACK_KS = _sdf->GetElement( "rrt_fb_ks" )->GetValueDouble();

    if( !_sdf->HasElement("rrt_fb_kd") ) {
      gzerr << "Unable to find parameter: rrt_fb_kd\n";
      return;
    } 
    FEEDBACK_KD = _sdf->GetElement( "rrt_fb_kd" )->GetValueDouble();

    if( !_sdf->HasElement("rrt_fb_du_speed") ) {
      gzerr << "Unable to find parameter: rrt_fb_du_speed\n";
      return;
    } 
    FEEDBACK_DU_SPEED = _sdf->GetElement( "rrt_fb_du_speed" )->GetValueDouble();

    if( !_sdf->HasElement("rrt_fb_du_angle") ) {
      gzerr << "Unable to find parameter: rrt_fb_du_angle\n";
      return;
    } 
    FEEDBACK_DU_ANGLE = _sdf->GetElement( "rrt_fb_du_angle" )->GetValueDouble();

    if( !_sdf->HasElement("rrt_fb_max_speed") ) {
      gzerr << "Unable to find parameter: rrt_fb_max_speed\n";
      return;
    } 
    FEEDBACK_MAX_SPEED = _sdf->GetElement( "rrt_fb_max_speed" )->GetValueDouble();

    if( !_sdf->HasElement("rrt_fb_max_angle") ) {
      gzerr << "Unable to find parameter: rrt_fb_max_angle\n";
      return;
    } 
    FEEDBACK_MAX_ANGLE = _sdf->GetElement( "rrt_fb_max_angle" )->GetValueDouble();

  } else if( PLANNER == LISSAJOUS ) {
    if( !_sdf->HasElement("lissajous_planner_step_size") ) {
      gzerr << "Unable to find parameter: lissajous_planner_step_size\n";
      return;
    } 
    PLANNER_STEP_SIZE = _sdf->GetElement( "lissajous_planner_step_size" )->GetValueDouble();

    if( !_sdf->HasElement("lissajous_steering_control_kp") ) {
      gzerr << "Unable to find parameter: lissajous_steering_control_kp\n";
      return;
    } 
    STEERING_CONTROL_KP = _sdf->GetElement( "lissajous_steering_control_kp" )->GetValueDouble();

    if( !_sdf->HasElement("lissajous_steering_control_kd") ) {
      gzerr << "Unable to find parameter: lissajous_steering_control_kd\n";
      return;
    } 
    STEERING_CONTROL_KD = _sdf->GetElement( "lissajous_steering_control_kd" )->GetValueDouble();

    if( !_sdf->HasElement("lissajous_speed_control_kp") ) {
      gzerr << "Unable to find parameter: lissajous_speed_control_kp\n";
      return;
    } 
    SPEED_CONTROL_KP = _sdf->GetElement( "lissajous_speed_control_kp" )->GetValueDouble();
  
    if( !_sdf->HasElement("lissajous_fb_ks") ) {
      gzerr << "Unable to find parameter: lissajous_fb_ks\n";
      return;
    } 
    FEEDBACK_KS = _sdf->GetElement( "lissajous_fb_ks" )->GetValueDouble();

    if( !_sdf->HasElement("lissajous_fb_kd") ) {
      gzerr << "Unable to find parameter: lissajous_fb_kd\n";
      return;
    } 
    FEEDBACK_KD = _sdf->GetElement( "lissajous_fb_kd" )->GetValueDouble();

    if( !_sdf->HasElement("lissajous_fb_du_speed") ) {
      gzerr << "Unable to find parameter: lissajous_fb_du_speed\n";
      return;
    } 
    FEEDBACK_DU_SPEED = _sdf->GetElement( "lissajous_fb_du_speed" )->GetValueDouble();

    if( !_sdf->HasElement("lissajous_fb_du_angle") ) {
      gzerr << "Unable to find parameter: lissajous_fb_du_angle\n";
      return;
    } 
    FEEDBACK_DU_ANGLE = _sdf->GetElement( "lissajous_fb_du_angle" )->GetValueDouble();

    if( !_sdf->HasElement("lissajous_fb_max_speed") ) {
      gzerr << "Unable to find parameter: lissajous_fb_max_speed\n";
      return;
    } 
    FEEDBACK_MAX_SPEED = _sdf->GetElement( "lissajous_fb_max_speed" )->GetValueDouble();

    if( !_sdf->HasElement("lissajous_fb_max_angle") ) {
      gzerr << "Unable to find parameter: lissajous_fb_max_angle\n";
      return;
    } 
    FEEDBACK_MAX_ANGLE = _sdf->GetElement( "lissajous_fb_max_angle" )->GetValueDouble();

  }

  // Steering Parameters
  if( !_sdf->HasElement("steering") ) {
    gzerr << "Unable to find parameter: steering\n";
    return;
  } 
  std::string steering = _sdf->GetElement( "steering" )->GetValueString();
  if( steering.compare("direct") == 0 ) {
    STEERING = DIRECT;
  } else if( planner.compare("fourbar") == 0 ) {
    STEERING = FOURBAR;
  } else {
    STEERING = FOURBAR;
  }
 
  // Double Four Bar Steering Linkage Parameters
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


  BASE_LINK_LENGTH = (kingpin_front_left->GetAnchor( 0 ) - steering_box->GetAnchor( 0 )).GetLength();

  // Compute Four Bar Steering Linkage Helpers
  BASE_LINK_LENGTH_SQ = BASE_LINK_LENGTH * BASE_LINK_LENGTH;
  STEERING_LEVER_LENGTH_SQ = STEERING_LEVER_LENGTH * STEERING_LEVER_LENGTH;
  TIEROD_LENGTH_SQ = TIEROD_LENGTH * TIEROD_LENGTH;
  SPINDLE_LEVER_LENGTH_SQ = SPINDLE_LEVER_LENGTH * SPINDLE_LEVER_LENGTH;
 
  WHEEL_BASE = (steering_link->GetWorldPose().pos - rear_driveshaft->GetWorldPose().pos).GetLength();

  std::cout << WHEEL_BASE << std::endl;

  // Derive Four Bar Steering Linkage Variables
  Real theta = STEERING_LEVER_BASE_ANGLE;
  Real h_sq = BASE_LINK_LENGTH_SQ + STEERING_LEVER_LENGTH_SQ - 
          2.0 * BASE_LINK_LENGTH * STEERING_LEVER_LENGTH * cos( theta );
  Real h = sqrt( h_sq );
  Real rho = acos( (BASE_LINK_LENGTH_SQ + h_sq - STEERING_LEVER_LENGTH_SQ) / 
          (2.0 * BASE_LINK_LENGTH * h) ) +
          acos( (SPINDLE_LEVER_LENGTH_SQ + h_sq - TIEROD_LENGTH_SQ) / 
          (2.0 * SPINDLE_LEVER_LENGTH * h) );
  SPINDLE_LEVER_BASE_ANGLE = PI - rho;


  if( !_sdf->HasElement("lissajous_planner_ks") ) {
    gzerr << "Unable to find parameter: lissajous_planner_ks\n";
    return;
  } 
  LISSAJOUS_KS = _sdf->GetElement( "lissajous_planner_ks" )->GetValueDouble();

  if( !_sdf->HasElement("lissajous_planner_kd") ) {
    gzerr << "Unable to find parameter: lissajous_planner_kd\n";
    return;
  } 
  LISSAJOUS_KD = _sdf->GetElement( "lissajous_planner_kd" )->GetValueDouble();

  if( !_sdf->HasElement("lissajous_planner_du_speed") ) {
    gzerr << "Unable to find parameter: lissajous_planner_du_speed\n";
    return;
  } 
  LISSAJOUS_DU_SPEED = _sdf->GetElement( "lissajous_planner_du_speed" )->GetValueDouble();

  if( !_sdf->HasElement("lissajous_planner_du_angle") ) {
    gzerr << "Unable to find parameter: lissajous_planner_du_angle\n";
    return;
  } 
  LISSAJOUS_DU_ANGLE = _sdf->GetElement( "lissajous_planner_du_angle" )->GetValueDouble();

  stopped = false;

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

  sense( );
  if( !plan( ) ) {
    //std::cout << "planning failed\n";
    time_last = time;
  }

  if( stopped ) 
    stop();
  else
    act( );

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
  audit_file_fb_commands = "car_fb_commands.txt";
  audit_file_ff_commands = "car_ff_commands.txt";
  audit_file_interp_states = "car_interp_states.txt";
  audit_file_fb_error = "car_fb_error_states.txt";
  audit_file_control_values = "car_control_values.txt";

  write_command_audit_header( audit_file_planned_commands );
  write_state_audit_header( audit_file_planned_states );
  write_command_audit_header( audit_file_actual_commands );
  write_state_audit_header( audit_file_actual_states );
  write_command_audit_header( audit_file_ff_commands );
  write_command_audit_header( audit_file_fb_commands );
  write_state_audit_header( audit_file_interp_states );
  write_state_audit_header( audit_file_fb_error );
  write_command_audit_header( audit_file_control_values );

  std::cerr << "Car Controller Plugin Started" << std::endl;

  time_start = world->GetSimTime().Double();
  time_last = time_start;
}

//-----------------------------------------------------------------------------
void car_c::sense( void ) {

}

//-----------------------------------------------------------------------------
bool car_c::plan( void ) {
  if( states.size() == 0 ) {
    if( PLANNER == LISSAJOUS ) {
      if( !plan_lissajous() ) return false;
    } else if( PLANNER == RRT ) {
      if( plan_via_ompl() ) return false;
    }
  }

  if( ff_command.duration <= 0.0 ) {
    //std::cout << "commands:" << commands.size() << std::endl;
    if( commands.size() == 0 ) return false;

    ff_command = commands.front();
    commands.erase( commands.begin() );

    write_audit_datum( audit_file_ff_commands, ff_command );
  }


///*
  if( desired_state_duration >= PLANNER_STEP_SIZE ) {
    update_desired_state( );
  }
  desired_state_duration += dtime;
 

  if( stopped ) return false;
//*/

/*
  if( time >= desired_state_1.time )
    update_desired_state( );
*/
  return true;
}


//-----------------------------------------------------------------------------
void car_c::act( void ) {

  ff_command.duration -= dtime;

  fb_command = compute_feedback_command( ); 
  write_audit_datum( audit_file_fb_commands, fb_command );

  //car_command_c cmd = ff_command;
  car_command_c cmd = ff_command + fb_command;
  cmd.duration = dtime;

  steer( cmd, STEERING_CONTROL_KP, STEERING_CONTROL_KD );
  push( cmd, SPEED_CONTROL_KP );

  car_command_c actual_command( dtime, speed(), steering_angle() );
  car_state_c actual_state = state();

  write_audit_datum( audit_file_actual_commands, cmd );
  write_audit_datum( audit_file_actual_states, actual_state );
  write_audit_datum( audit_file_control_values, actual_command );
}
//-----------------------------------------------------------------------------
car_command_c car_c::compute_feedback_command( void ) {

  car_state_c qx = interpolate_linear(desired_state_0, desired_state_1, PLANNER_STEP_SIZE, ++state_step );
  car_state_c q = state();
  car_state_c dq = dstate();
  car_state_c dq_desired = qx - q;

  car_state_c dqx = FEEDBACK_KS * (qx - q) + FEEDBACK_KD * (dq_desired - dq);

  car_command_c cmd = inverse_ode( q, dqx );

  cmd.speed *= FEEDBACK_DU_SPEED;
  cmd.angle *= FEEDBACK_DU_ANGLE;
  cmd.duration = dtime;

  dq_desired.time = time;
  write_audit_datum( audit_file_fb_error, dq_desired );
  write_audit_datum( audit_file_interp_states, qx );

  if( fabs(cmd.speed) > FEEDBACK_MAX_SPEED )
    cmd.speed = sgn(cmd.speed) * FEEDBACK_MAX_SPEED;
  if( fabs(cmd.angle) > FEEDBACK_MAX_ANGLE )
    cmd.angle = sgn(cmd.angle) * FEEDBACK_MAX_ANGLE;

  return cmd;
}

//-----------------------------------------------------------------------------
void car_c::update_desired_state( void ) {
  if( stopped ) return;

  desired_state_0 = desired_states.front();
  desired_state_duration = 0.0;
  desired_states.erase( desired_states.begin() );
  if( !desired_states.size() ) {
    std::cout << "end of planned path\n";
    std::cout << "stopping car\n";
    stopped = true;
  }
  desired_state_1 = desired_states.front();
  // if front == end path is done!  Candidate trigger for replanning
  state_step = 0;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
car_state_c car_c::state( void ) {
  gazebo::math::Vector3 pos = rear_driveshaft->GetWorldPose().pos;
  gazebo::math::Quaternion rot = model->GetWorldPose().rot;
  Real x = pos.x;
  Real y = pos.y;
  Real theta = rot.GetYaw();
  Real t = time;
  return car_state_c( t, x, y, theta );
}
//-----------------------------------------------------------------------------
car_state_c car_c::dstate( void ) {
  return state() - prev_state;
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
  Real phi = steering_box->GetAngle( 0 ).Radian();
  while( phi > PI )
    phi -= 2.0 * PI;
  while( phi <= -PI )
    phi += 2.0 * PI;
  return phi;
}

//-----------------------------------------------------------------------------
Real car_c::speed( void ) {
  return model->GetRelativeLinearVel().x;
}

//-----------------------------------------------------------------------------
Real car_c::acceleration( void ) {
  return model->GetRelativeLinearAccel().x;
}

//-----------------------------------------------------------------------------
void car_c::steer( const car_command_c& u, const Real& Kp, const Real& Kd ) {

  if( STEERING == FOURBAR ) {
    steer_double_four_bar( steering_angle() );
  } else if( STEERING == DIRECT ) {
    steer_direct( steering_angle() );
  } else {
    steer_direct( steering_angle() );
  }

  Real desired_angle = u.angle;
  Real actual_angle = steering_angle();

  Real desired_velocity = MAX_STEERING_VELOCITY;
  Real actual_velocity = steering_box->GetVelocity( 0 );

  Real torque = Kp * ( desired_angle - actual_angle ) + 
                Kd * ( desired_velocity - actual_velocity );

  steering_box->SetForce( 0, torque );

}
//-----------------------------------------------------------------------------
void car_c::push( const car_command_c& u, const Real& Kp ) {

  Real desired_speed = u.speed;
  Real actual_speed = speed();

  Real f = Kp * ( desired_speed - actual_speed );

  gazebo::math::Vector3 d = orientation( );
  gazebo::math::Vector3 v = d * f;
  body->AddForce( v );
} 

//-----------------------------------------------------------------------------
void car_c::stop( void ) {
  //bearing_rear_left->SetForce( 0, 0.0 );
  //bearing_rear_right->SetForce( 0, 0.0 );
  car_command_c cmd( 0.0, 0.0 );
  steer( cmd, STEERING_CONTROL_KP, STEERING_CONTROL_KD );
  push( cmd, SPEED_CONTROL_KP );
}

//-----------------------------------------------------------------------------
void car_c::steer_direct( const Real& _steering_angle ) {

  kingpin_front_left->SetAngle( 0, gazebo::math::Angle( _steering_angle ) );
  kingpin_front_right->SetAngle( 0, gazebo::math::Angle( _steering_angle ) );

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
car_state_c car_c::ode( const car_state_c& q, const car_command_c& u ) {
  Real dx = u.speed * cos( q.theta );
  Real dy = u.speed * sin( q.theta );
  Real dtheta = u.speed * tan( u.angle ) / WHEEL_BASE;

  return car_state_c( dx, dy, dtheta );
}


//------------------------------------------------------------------------------
Real car_c::speed_optimization_function( const Real& u_s, const Real& dx, const Real& dy, const Real& costheta, const Real& sintheta ) {
  return u_s * u_s + dx * dx + dy * dy - 2.0 * u_s * ( dx * costheta + dy * sintheta );
}

//------------------------------------------------------------------------------
car_state_c car_c::interpolate_linear( const car_state_c& q0, const car_state_c& qf, const Real& deltat, const int& step ) {
  return ((qf - q0)) * (deltat * (Real)step) + q0;
}

//------------------------------------------------------------------------------
car_command_c car_c::inverse_ode(const car_state_c& q, const car_state_c& dq ) {

  Real u_s, u_phi;

  // *First Compute Speed*
  // Attempt optimization of u_s using backtracking line search
  // Note: optimization is necessary because the reality is that dx/cos rarely 
  // equals dy/sin and by switching between sin and cos which must be done else 
  // DIV0, discontinuities are introduced that can be substantial changes.
  Real c = cos( q.theta );
  Real s = sin( q.theta );

  if( fabs(c) > 0.5 )
    u_s = dq.x / c;
  else
    u_s = dq.y / s;
  
  Real du, f0, f, d1, t;
  const Real alpha = 0.01;
  const Real beta = 0.5;
  int its = 0;
  const int MAX_ITS = 1000;
  const Real EPSILON = 1e-3;
  const Real EPSILON2 = 1e-3; 

  do{
    t = 1.0;
    d1 = 2.0 * u_s - 2.0 * ( dq.x * c + dq.y * s );
    du = -d1/2.0;
    f0 = speed_optimization_function(u_s, dq.x, dq.y, c, s );
    f = speed_optimization_function( u_s + t * du, dq.x, dq.y, c, s );
    while( f > f0 + alpha * t * d1 * du ) {
      t = beta * t;
      f = speed_optimization_function( u_s + t * du, dq.x, dq.y, c, s );
    }
    u_s = u_s + t * du;
  
  } while(its++ < MAX_ITS && f > EPSILON && fabs(du) > EPSILON2 );
  //} while(its++ < MAX_ITS && f0 > EPSILON && fabs(du) > EPSILON2 );

  // *Second Compute Steering Angle*
  u_phi = atan2( dq.theta * WHEEL_BASE, u_s );

  return car_command_c( u_s, u_phi );
}

//------------------------------------------------------------------------------
/*
void car_c::compensate_for_roll_pitch( void ) {
  // For tight turns, this ensures that the model sticks to the ground
  // and doesn't roll over.  It can lead to skidding
  gazebo::math::Quaternion rot = model->GetWorldPose().rot;
  //Real _roll = rot.GetRoll();
  //Real _pitch = rot.GetPitch();
  Real _yaw = rot.GetYaw();
  gazebo::math::Vector3 pos = model->GetWorldPose().pos;
  gazebo::math::Quaternion rot_fixed( 0.0, 0.0, _yaw );
  model->SetWorldPose( gazebo::math::Pose( pos, rot_fixed ) );
}
*/
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

  steer( command, STEERING_CONTROL_KP, STEERING_CONTROL_KD );
  push( command, SPEED_CONTROL_KP );
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
bool car_c::plan_lissajous( void ) {
  //Real dt = 0.025;  // Note: this is the highest possible step size before loss of stability
  Real dt = PLANNER_STEP_SIZE;
  Real Ks = LISSAJOUS_KS;
  Real Kd = LISSAJOUS_KD;
  Real DU_speed = LISSAJOUS_DU_SPEED;
  Real DU_angle = LISSAJOUS_DU_ANGLE;
  Real period = 2 * PI;
  car_state_c initial( 4.0, 0.0, PI/2.0 );
  state( initial );
  car_state_c dq( 0.0, 0.0, 0.0 );
  car_state_c q = state();
  states.push_back( q );
  
  write_audit_datum( audit_file_planned_states, q );

  Real alpha = 10.0;
  for( Real t = dt; t <= period * alpha + dt; t += dt ) {
    car_state_c q_desired = lissajous( t / alpha, 1.0, 2.0, 4.0, 4.0, PI/2.0 );
    q = state();
    car_state_c dq_desired = q_desired - q;
    car_state_c delta_q = Ks * (q_desired - q) + Kd * (dq_desired - dq);
    car_command_c u = inverse_ode( q, delta_q );

    u.speed *= DU_speed;
    u.angle *= DU_angle;
    u.duration = dt ;

    dq = ode( q, u );

    q = q + dq * dt;

    state( q );

    commands.push_back( u );
    write_audit_datum( audit_file_planned_commands, u );
    states.push_back( q );
    q.time = t;
    desired_states.push_back( q );
    write_audit_datum( audit_file_planned_states, q );
  }
  update_desired_state( );
  state( initial );
  prev_state = car_state_c( initial );

  return true;
}

//-----------------------------------------------------------------------------
// OMPL
//-----------------------------------------------------------------------------

bool car_c::plan_via_ompl(void) {
  /// construct the state space we are planning in
  ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());

  /// set the bounds for the R^2 part of SE(2)
  ompl::base::RealVectorBounds bounds(2);
  bounds.setLow( 0, -25.0 );  // min x
  bounds.setHigh( 0, 25.0 );  // max x
  bounds.setLow( 1, -25.0 );  // min y
  bounds.setHigh( 1, 25.0 );  // max y

  space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

  // create a control space
  ompl::control::ControlSpacePtr cspace(new demo_control_space_c(space));

  const Real MAX_SPEED = 0.5;

  // set the bounds for the control space
  ompl::base::RealVectorBounds cbounds(2);
  //const Real max_steering_angle = PI/5.0;
  cbounds.setLow( 0, -MAX_SPEED );
  cbounds.setHigh( 0, MAX_SPEED );
  cbounds.setLow( 1, -MAX_STEERING_ANGLE );
  cbounds.setHigh( 1, MAX_STEERING_ANGLE );

  cspace->as<demo_control_space_c>()->setBounds(cbounds);

  // define a simple setup class
  ompl::control::SimpleSetup ss(cspace);

  ompl::control::SpaceInformationPtr si( ss.getSpaceInformation() );
  ompl::base::ProblemDefinition pdef(si);

  /// set state validity checking for this space
  ss.setStateValidityChecker(boost::bind(&is_state_valid, ss.getSpaceInformation().get(), _1));

  /// set the propagation routine for this space
  ss.setStatePropagator( ompl::control::StatePropagatorPtr( new demo_state_propagator_c( ss.getSpaceInformation(), MAX_STEERING_ANGLE, WHEEL_BASE ) ) );

  car_state_c initial( 0.0, 4.0, 0.0 );
  car_state_c final( 0.0, 0.0, 0.0 );
  state( initial );
  //active_state = car_state_c( initial );
  prev_state = car_state_c( initial );

  /// create a start state
  ompl::base::ScopedState<ompl::base::SE2StateSpace> start(space);
  start->setX( initial.x );
  start->setY( initial.y );
  start->setYaw( initial.theta );

  /// create a  goal state; use the hard way to set the elements
  ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(space);
  goal->setX( final.x );
  goal->setY( final.y );
  goal->setYaw( final.theta );

  const Real INTEGRATOR_STEP_SIZE = 0.05;
  const Real MAX_PLANNING_TIME = 10.0;

  /// set the start and goal states
  ss.setStartAndGoalStates(start, goal, INTEGRATOR_STEP_SIZE);

  //ompl::base::PlannerPtr planner( new ompl::control::RRT(si) );
  ompl::control::RRT* rrt = new ompl::control::RRT(si);
  // rrt->setGoalBias( dbl );
  // rrt->setIntermediateStates( true );
  ompl::base::PlannerPtr planner( rrt );


  ss.setPlanner( planner );

  ss.getSpaceInformation()->setPropagationStepSize( PLANNER_STEP_SIZE );
  //ss.getSpaceInformation()->setPropagationStepSize( 0.001 );
  //ss.getSpaceInformation()->setPropagationStepSize( 0.025 );
  /// we want to have a reasonable value for the propagation step size
  ss.setup();
  static_cast<demo_state_propagator_c*>(ss.getStatePropagator().get())->setIntegrationTimeStep(ss.getSpaceInformation()->getPropagationStepSize());

  /// attempt to solve the problem
  ompl::base::PlannerStatus solved = ss.solve(MAX_PLANNING_TIME);

  if (solved) {

    //ss.haveExactSolutionPath();  // maybe true
    //ss.haveSolutionPath();       // true

    std::cout << "Found solution:" << std::endl;
    /// print the path to screen

    //ss.getSolutionPath().asGeometric().print(std::cout);

    ompl::geometric::PathGeometric path = ss.getSolutionPath().asGeometric();
    unsigned int state_count = path.getStateCount();

    const Real EPSILON = 0.5;
    // check that final state is very close to goal state
    ompl::base::State* state = path.getState( state_count - 1 );
    Real x = state->as<ompl::base::SE2StateSpace::StateType>()->getX();
    Real y = state->as<ompl::base::SE2StateSpace::StateType>()->getY();
    Real theta = state->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
    if( fabs( x - final.x ) > EPSILON || fabs( y - final.y ) > EPSILON || fabs( theta - final.theta ) > EPSILON ) {
      std::cout << "Planning Failed: final state diverged significantly from goal state\n";
      if( fabs( x - final.x ) > EPSILON ) {
        std::cout << "x diff: " << x - final.x << std::endl;
      }
      if( fabs( y - final.y ) > EPSILON ) {
        std::cout << "y diff: " << y - final.y << std::endl;
      }
      if( fabs( theta - final.theta ) > EPSILON ) {
        std::cout << "theta diff: " << theta - final.theta << std::endl;
      }
      
      return false;
    }
    

    for( unsigned int i = 0; i < state_count; i++ ) {
      ompl::base::State* state = path.getState( i );
      Real x = state->as<ompl::base::SE2StateSpace::StateType>()->getX();
      Real y = state->as<ompl::base::SE2StateSpace::StateType>()->getY();
      Real theta = state->as<ompl::base::SE2StateSpace::StateType>()->getYaw();

      car_state_c q( x, y, theta );
      states.push_back( q );
      q.time = (Real)i * PLANNER_STEP_SIZE;
      desired_states.push_back( q );
      write_audit_datum( audit_file_planned_states, q );
    }
    update_desired_state( );

    unsigned int control_count = ss.getSolutionPath().getControlCount();
    for( unsigned int i = 0; i < control_count; i++ ) {
      ompl::control::Control* control = ss.getSolutionPath().getControl( i );
      double duration = ss.getSolutionPath().getControlDuration( i );
      Real speed = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0];
      Real phi = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1];
 
      car_command_c u( duration, speed, phi );
      commands.push_back( u );
      write_audit_datum( audit_file_planned_commands, u );
    }

    return true;
  } else {
    std::cout << "No solution found" << std::endl;
    return false;
  }
}

//-----------------------------------------------------------------------------

/// implement the function describing the robot motion: qdot = f(q, u)
void car_c::operator()(const ompl::base::State *state, const ompl::control::Control *control, std::valarray<double> &dstate, const double& wheel_base ) const {
  const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
  const double theta = state->as<ompl::base::SE2StateSpace::StateType>()->getYaw();

  dstate.resize(3);
  dstate[0] = u[0] * cos(theta);
  dstate[1] = u[0] * sin(theta);
  dstate[2] = u[0] * tan(u[1]) / wheel_base;
}
//-----------------------------------------------------------------------------

/// implement y(n+1) = y(n) + d
void car_c::update(ompl::base::State *state, const std::valarray<double> &dstate) const {
  ompl::base::SE2StateSpace::StateType &s = *state->as<ompl::base::SE2StateSpace::StateType>();
  s.setX(s.getX() + dstate[0]);
  s.setY(s.getY() + dstate[1]);
  s.setYaw(s.getYaw() + dstate[2]);
  space_->enforceBounds(state);
}
//-----------------------------------------------------------------------------

