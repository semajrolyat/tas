#ifndef _SENSOR_H_
#define _SENSOR_H_

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

class ship_sensor_c {
public:
  
  gazebo::physics::ModelPtr model;

  ship_sensor_c( void ) {}
  ship_sensor_c( gazebo::physics::ModelPtr _model ) : model( _model ) {}

  virtual ~ship_sensor_c( void ) {}
  
  // If gazebo
  bool sense( ship_state_c& q ) {
    gazebo::math::Vector3 pos = model->GetWorldPose().pos;
    gazebo::math::Quaternion rot = model->GetWorldPose().rot;
    gazebo::math::Vector3 dpos = model->GetWorldLinearVel();
    gazebo::math::Vector3 drot = model->GetWorldAngularVel();

    Ravelin::Vector3d x( pos.x, pos.y, pos.z );
    Ravelin::Quatd theta( rot.x, rot.y, rot.z, rot.w );
    Ravelin::Vector3d dx( dpos.x, dpos.y, dpos.z );
    Ravelin::Vector3d dtheta( drot.x, drot.y, drot.z );

    q = ship_state_c( x, theta, dx, dtheta );

    return true;
  }
};

//-----------------------------------------------------------------------------

#endif // _SENSOR_H_

