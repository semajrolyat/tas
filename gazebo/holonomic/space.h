#ifndef _GAZEBO_SPACE_H_
#define _GAZEBO_SPACE_H_

//-----------------------------------------------------------------------------

#include "plane.h"
#include "aabb.h"

//-----------------------------------------------------------------------------

#include <stdlib.h>
#include <iostream>
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

#include <boost/shared_ptr.hpp>

//-----------------------------------------------------------------------------

class space_c {
public:
  // the gazebo reference to the world in which the ship is located
  gazebo::physics::WorldPtr world;

  plane_list_t planes;

  aabb_c bounds;

  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
  space_c( void );
  space_c( gazebo::physics::WorldPtr world );
    
  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~space_c( void );

  //---------------------------------------------------------------------------
  void read( void );

  //---------------------------------------------------------------------------
  static void make_maze( void );
};

//-----------------------------------------------------------------------------

#endif // _GAZEBO_SPACE_H_

