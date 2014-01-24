#ifndef _SPACE_H_
#define _SPACE_H_

//-----------------------------------------------------------------------------

#include "common.h"
#include "plane.h"
#include "aabb.h"

//-----------------------------------------------------------------------------

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>
#include <limits>

#if defined(GAZEBO_DYNAMICS)
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#elif defined(MOBY_DYNAMICS)
#endif

#include <boost/shared_ptr.hpp>

//-----------------------------------------------------------------------------

class space_c {
public:
#if defined(GAZEBO_DYNAMICS)
  // the gazebo reference to the world in which the ship is located
  gazebo::physics::WorldPtr world;
#endif

  plane_list_t planes;

  aabb_c bounds;

  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
  space_c( void );
#if defined(GAZEBO_DYNAMICS)
  space_c( gazebo::physics::WorldPtr world );
#endif
  space_c( std::string serial );
    
  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~space_c( void );

  //---------------------------------------------------------------------------
  bool read( void );
  std::string serialize( void );

#if defined(GAZEBO_DYNAMICS)
  //---------------------------------------------------------------------------
  static sdf::SDF make_maze( void );
#endif
};

//-----------------------------------------------------------------------------

#endif // _SPACE_H_

