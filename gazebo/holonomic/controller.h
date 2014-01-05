#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

//-----------------------------------------------------------------------------
#include "common.h"
#include "constants.h"
#include "aabb.h"
#include "command.h"
#include "state.h"
#include "utilities.h"
#include "space.h"

//-----------------------------------------------------------------------------
// Simulator
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
/*
//-----------------------------------------------------------------------------
// PDF
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multifit_nlin.h>
*/
//-----------------------------------------------------------------------------
// Math
#include <Ravelin/Pose3d.h>
#include <Ravelin/SVelocityd.h>
#include <Ravelin/SAcceld.h>
#include <Ravelin/SForced.h>
#include <Ravelin/SpatialRBInertiad.h>
#include <Ravelin/MatrixNd.h>

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

class ship_controller_c {
public:
  gazebo::physics::LinkPtr body;

  ship_controller_c( void ) {}
  ship_controller_c( gazebo::physics::LinkPtr _body ) {
    body = _body;
  }
  virtual ~ship_controller_c( void ) {}

  //---------------------------------------------------------------------------
  /// Command control interface.  Forwards commands to hardware controllers
  void control( const double& t, ship_command_c& u ) {
    control_position( t, u );
    control_rotation( t, u );
  }

  //---------------------------------------------------------------------------
  /// Position controller
  void control_position( const double& t, ship_command_c& u ) {
    // clear the accumulators
    gazebo::math::Vector3 f = -body->GetRelativeForce();
    body->AddRelativeForce( f );

    // add the command force
    body->AddForce( gazebo::math::Vector3( u.force().x(), u.force().y(), u.force().z() ) );
  }

  //---------------------------------------------------------------------------
  /// Rotation controller
  void control_rotation( const double& t, ship_command_c& u ) {
    // clear the accumulators
    gazebo::math::Vector3 tau = -body->GetRelativeTorque();
    body->AddRelativeTorque( tau );

    // add the command torque
    body->AddTorque( gazebo::math::Vector3( u.torque().x(), u.torque().y(), u.torque().z() ) );
  }
};

//-----------------------------------------------------------------------------

#endif // _CONTROLLER_H_

