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

// the integrator for the predator and the prey
template<typename F>
class pp_integrator_c {
private:
  const ompl::base::StateSpace *space;
  double time_step;
  F      ode;

  //---------------------------------------------------------------------------
public:
  pp_integrator_c( const ompl::base::StateSpace *_space, const double& _time_step ) : 
    space( _space ), 
    time_step( _time_step), 
    ode( _space )
  { }

  //---------------------------------------------------------------------------
  void propagate( const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result, ship_c* pred, ship_c* prey ) const {
    std::vector<double> state_pred, state_prey, dstate_pred, dstate_prey;
    std::vector<double> u_pred, u_prey;
    space->copyState( result, start );

    // get the predator and prey states
    pp_state_c q( space, start );

    // TODO: get the predator and prey states

    // get the commands for the predator and prey
    ship_command_c u( control );
    std::vector<double> u_pred = u.as_vector();
    std::vector<double> u_prey;
    ship::prey_command(state_pred, state_prey, u_prey);

    // get the ODEs for the predator and prey
    pred->ode(state_pred, u_pred, state_dpred);
    prey->ode(state_prey, u_prey, state_dprey);

    // update the predator and prey states
    for (unsigned i=0; i< state_pred.size(); i++)
    {
      state_pred[i] += state_dpred[i] * time_step;
      state_prey[i] += state_dprey[i] * time_step;
    }

    // renormalize the quaternions
    gazebo::math::Quaternion ed(state_pred[6], state_pred[3], state_pred[4], state_pred[5]);
    gazebo::math::Quaternion ey(state_prey[6], state_prey[3], state_prey[4], state_prey[5]);
    ed.Normalize();
    ey.Normalize();
    state_pred[6] = ed.x;
    state_pred[3] = ed.y;
    state_pred[4] = ed.z;
    state_pred[5] = ed.w;
    state_prey[6] = ey.x;
    state_prey[3] = ey.y;
    state_prey[4] = ey.z;
    state_prey[5] = ey.w;

    // TODO: convert back to the state space


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


#endif // _GAZEBO_SHIP_INTEGRATOR_H_

