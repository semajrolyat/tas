#ifndef _GAZEBO_SHIP_CONTROL_SPACE_H_
#define _GAZEBO_SHIP_CONTROL_SPACE_H_

#include <ompl/base/StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

//-----------------------------------------------------------------------------

class control_space_c : public ompl::control::RealVectorControlSpace {
public:
  control_space_c(const ompl::base::StateSpacePtr &stateSpace) : 
    ompl::control::RealVectorControlSpace(stateSpace, 6)
  { }
};

//-----------------------------------------------------------------------------

#endif // _GAZEBO_SHIP_CONTROL_SPACE_H_

