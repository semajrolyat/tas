#ifndef _SENSOR_H_
#define _SENSOR_H_

//-----------------------------------------------------------------------------

#include "common.h"
#include "state.h"
/*
#include "constants.h"
#include "aabb.h"
#include "command.h"
#include "utilities.h"
#include "space.h"
*/
/*
//-----------------------------------------------------------------------------
// Simulator
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
*/
#include <Ravelin/Vector3d.h>

#if defined(GAZEBO_DYNAMICS)
 #include <gazebo/physics/Model.hh>
#elif defined(MOBY_DYNAMICS)
 #include <Moby/RigidBody.h>
 #include <Ravelin/SVelocityd.h>
#endif

//-----------------------------------------------------------------------------

class sensor_c {
private:
 
 #if defined(GAZEBO_DYNAMICS)
  gazebo::physics::ModelPtr model;
 #elif defined(MOBY_DYNAMICS)
  Moby::RigidBodyPtr body;
 #endif

  //---------------------------------------------------------------------------
public:

  //---------------------------------------------------------------------------
  sensor_c( void ) {}

  //---------------------------------------------------------------------------
 #if defined(GAZEBO_DYNAMICS)
  sensor_c( gazebo::physics::ModelPtr _model ) { model =  _model; }

  //---------------------------------------------------------------------------
 #elif defined(MOBY_DYNAMICS)
  sensor_c( Moby::RigidBodyPtr _body ) { body = _body; }
 #endif

  //---------------------------------------------------------------------------
  virtual ~sensor_c( void ) {}

  //---------------------------------------------------------------------------
  bool sense( ship_state_c& q ) {
   #if defined(GAZEBO_DYNAMICS)
    gazebo::math::Vector3 pos = model->GetWorldPose().pos;
    gazebo::math::Quaternion rot = model->GetWorldPose().rot;
    gazebo::math::Vector3 dpos = model->GetWorldLinearVel();
    gazebo::math::Vector3 drot = model->GetWorldAngularVel();

    Ravelin::Vector3d x( pos.x, pos.y, pos.z );
    Ravelin::Quatd theta( rot.x, rot.y, rot.z, rot.w );
    Ravelin::Vector3d dx( dpos.x, dpos.y, dpos.z );
    Ravelin::Vector3d dtheta( drot.x, drot.y, drot.z );

    q = ship_state_c( x, theta, dx, dtheta );
   #elif defined(MOBY_DYNAMICS)
    Ravelin::Pose3d pose = body->get_pose();
    Ravelin::SVelocityd dpose = body->get_velocity();

    Ravelin::Vector3d x( pose.x.x(), pose.x.y(), pose.x.z() );
    Ravelin::Quatd theta( pose.q.x, pose.q.y, pose.q.z, pose.q.w );
    Ravelin::Vector3d dx( dpose[0], dpose[1], dpose[2] );
    Ravelin::Vector3d dtheta( dpose[3], dpose[4], dpose[5] );

    q = ship_state_c( x, theta, dx, dtheta );
   #endif
    return true;
  }
};

//-----------------------------------------------------------------------------

#endif // _SENSOR_H_

