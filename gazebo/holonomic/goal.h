#ifndef _GAZEBO_SHIP_GOAL_H_
#define _GAZEBO_SHIP_GOAL_H_

//-----------------------------------------------------------------------------

#include "state.h"

#include <ompl/base/State.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>

//-----------------------------------------------------------------------------

class ship_goal_c : public ompl::base::GoalState {
public:
  const ompl::base::StateSpace *space;
  double SATISFACTION_THRESHOLD;

  //---------------------------------------------------------------------------
  ship_goal_c( const ompl::base::SpaceInformationPtr& si ) : 
    ompl::base::GoalState(si),
    space( si->getStateSpace().get() )
  {
    SATISFACTION_THRESHOLD = 1e-4;

  }

  //---------------------------------------------------------------------------
  virtual bool isSatisfied( const ompl::base::State* state ) const {
    ship_state_c q( space, state );
    if( distanceGoal( state ) < SATISFACTION_THRESHOLD ) return true;
    return false;
  }

  //---------------------------------------------------------------------------
  virtual bool isSatisfied( const ompl::base::State* state, double *distance ) const {
    ship_state_c q( space, state );

    *distance = distanceGoal( state );
    if( *distance < SATISFACTION_THRESHOLD ) return true;
    return false;
  }

  //---------------------------------------------------------------------------
  virtual double distanceGoal( const ompl::base::State* state ) const {
    std::vector<double> g( ship_state_c::size() );
    for( unsigned i=0; i< ship_state_c::size(); i++ )
      g[i] = value(i);
    ship_state_c q( space, state );
    std::vector<double> d( ship_state_c::size() );
    for( unsigned i = 0; i < d.size(); i++ ) 
      d[i] = q.value(i) - g[i];

    return sqrt( d[0] * d[0] + d[1] * d[1] + d[2] * d[2] );
  }

  //---------------------------------------------------------------------------
  double value( const unsigned int i ) const {
    assert( i < ship_state_c::size() );

    return *space->getValueAddressAtIndex(state_, i);
  }
};

//-----------------------------------------------------------------------------

#endif // _GAZEBO_SHIP_GOAL_H_

