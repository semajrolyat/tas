/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

actuator.h
-----------------------------------------------------------------------------*/

#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

//-----------------------------------------------------------------------------

#include "common.h"

#if defined(GAZEBO_DYNAMICS)
 #include <gazebo/gazebo.hh>
 //#include <gazebo/common/Plugin.hh>
 //#include <gazebo/common/common.hh>
 //#include <gazebo/common/Events.hh>
 //#include <gazebo/physics/physics.hh>
#elif defined(MOBY_DYNAMICS)
 #include <Moby/Simulator.h>
 #include <Moby/RigidBody.h>
#endif

#include <Ravelin/Vector3d.h>

//-----------------------------------------------------------------------------

class actuator_c {
private:

 #if defined(GAZEBO_DYNAMICS)
  gazebo::physics::LinkPtr link;
 #elif defined(MOBY_DYNAMICS)
  Moby::RigidBodyPtr body;
 #endif

public:
  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  actuator_c( void ) { }

  //---------------------------------------------------------------------------
 #if defined(GAZEBO_DYNAMICS)
  actuator_c( gazebo::physics::LinkPtr _link ) { link = _link; }

  //---------------------------------------------------------------------------
 #elif defined(MOBY_DYNAMICS)
  actuator_c( Moby::RigidBodyPtr _body ) { body = _body; }
 #endif

  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  virtual ~actuator_c( void ) { }

  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  void add_force( const Ravelin::Vector3d& f ) {
   #if defined(GAZEBO_DYNAMICS)
    link->AddForce( gazebo::math::Vector3(f.x(), f.y(), f.z()) ); 
   #elif defined(MOBY_DYNAMICS)
    Ravelin::SForced force( f.x(), f.y(), f.z(), 0.0, 0.0, 0.0 );
    body->add_force( force );
   #endif
  }
 
  //---------------------------------------------------------------------------
  void add_torque( const Ravelin::Vector3d& tau ) {
   #if defined(GAZEBO_DYNAMICS)
    link->AddTorque( gazebo::math::Vector3(tau.x(), tau.y(), tau.z()) ); 
   #elif defined(MOBY_DYNAMICS)
    Ravelin::SForced force( 0.0, 0.0, 0.0, tau.x(), tau.y(), tau.z() );
    body->add_force( force );
   #endif
  }

  //---------------------------------------------------------------------------
  void clear_accumulators( void ) {
   #if defined(GAZEBO_DYNAMICS)
    gazebo::math::Vector3 f = -link->GetRelativeForce();
    link->AddRelativeForce( f )
    gazebo::math::Vector3 tau = -link->GetRelativeTorque();
    link->AddRelativeTorque( tau )
   #elif defined(MOBY_DYNAMICS)
    body->reset_accumulators();
   #endif
  }

};

//-----------------------------------------------------------------------------

#endif // _ACTUATOR_H_
