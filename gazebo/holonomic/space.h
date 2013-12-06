#ifndef _GAZEBO_SPACE_H_
#define _GAZEBO_SPACE_H_

//-----------------------------------------------------------------------------

#include <stdlib.h>
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

//-----------------------------------------------------------------------------

class space_c : public gazebo::ModelPlugin {
protected:

  //gazebo::event::ConnectionPtr updateConnection;

  gazebo::physics::ModelPtr model;
  gazebo::physics::WorldPtr world;
  //gazebo::physics::LinkPtr body;

  //double time, dtime, time_start, time_last;

public:
  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
  space_c( void );
    
  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~space_c( void );

protected:
  //---------------------------------------------------------------------------
  // Gazebo ModelPlugin Interface
  //---------------------------------------------------------------------------
  virtual void Load( gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf );
  //virtual void Update( );
};

//-----------------------------------------------------------------------------

#endif // _GAZEBO_SPACE_H_

