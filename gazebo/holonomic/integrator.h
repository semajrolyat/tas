#ifndef _GAZEBO_SHIP_INTEGRATOR_H_
#define _GAZEBO_SHIP_INTEGRATOR_H_

#include "command.h"
#include "state.h"
#include "utilities.h"

#include <valarray>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/control/Control.h>

//-----------------------------------------------------------------------------

class ship_c;           // forward declaration

//-----------------------------------------------------------------------------
template<typename F>
class euler_integrator_c {
private:
  const ompl::base::StateSpace *space;
  double time_step;
  F      ode;

  //---------------------------------------------------------------------------
public:
  euler_integrator_c( const ompl::base::StateSpace *_space, const double& _time_step ) : 
    space( _space ), 
    time_step( _time_step), 
    ode( _space )
  { }

  //---------------------------------------------------------------------------
  void propagate( const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result, ship_c* ship ) const {
    std::vector<double> dstate;
    space->copyState( result, start );

    ship_state_c q( space, start );
    ship_command_c u( control );

    ode( result, control, dstate, ship );
    ode.update( result, time_step * dstate );
/*
    std::valarray<double> dstate;
    space->copyState( result, start );

    ship_state_c q( space, start );
    ship_command_c u( control );

    ode( result, control, dstate, ship );
    ode.update( result, time_step * dstate );
*/
  }

  //---------------------------------------------------------------------------
  double get_time_step( void ) const {
    return time_step;
  }

  //---------------------------------------------------------------------------
  void set_time_step( const double& _time_step ) {
    time_step = _time_step;
  }
};

//-----------------------------------------------------------------------------

#endif // _GAZEBO_SHIP_INTEGRATOR_H_

