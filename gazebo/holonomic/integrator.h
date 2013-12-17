#ifndef _INTEGRATOR_H_
#define _INTEGRATOR_H_

#include "command.h"
#include "state.h"
#include "utilities.h"
#include "ship.h"

#include <valarray>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/control/Control.h>

//-----------------------------------------------------------------------------

template<typename F>
class euler_integrator_c {
private:
  ompl::base::StateSpace *statespace;
  double time_step;
  F      ode;

  //---------------------------------------------------------------------------
public:
  euler_integrator_c( ompl::base::StateSpace *_statespace, const double& _time_step ) : 
    statespace( _statespace ), 
    time_step( _time_step), 
    ode( _statespace )
  { }

  //---------------------------------------------------------------------------
  void propagate( const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result, ship_c* ship ) const {
    std::vector<double> dstate;
    statespace->copyState( result, start );

    ship_state_c q( statespace, start );
    ship_command_c u( control );

    ode( result, control, dstate, ship );
    ode.update( result, time_step * dstate );
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
  ompl::base::StateSpace *statespace;
  double time_step;
  F      ode;

  //---------------------------------------------------------------------------
public:
  pp_integrator_c( ompl::base::StateSpace *_statespace, const double& _time_step ) : 
    statespace( _statespace ), 
    time_step( _time_step), 
    ode( _statespace )
  { }

  //---------------------------------------------------------------------------
  void propagate( const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result, ship_c* pred, ship_c* prey ) const {
    std::vector<double> state_pred, state_prey, dstate_pred, dstate_prey;
    std::vector<double> u_pred, u_prey;
    statespace->copyState( result, start );

    // get the predator and prey states
    pp_state_c qi( statespace, start );
    state_pred = qi.pred_vector();
    state_prey = qi.prey_vector();

    // get the commands for the predator and prey
    ship_command_c u( control );
    u_pred = u.as_vector();
    ship_c::compute_prey_command(state_pred, state_prey, u_prey, pred->time, pred->dtime, &pred->space);

    // get the ODEs for the predator and prey
    ship_c::ode( state_pred, u_pred, dstate_pred, pred );
    ship_c::ode( state_prey, u_prey, dstate_prey, prey );

    // update the predator and prey states
    for (unsigned i=0; i< state_pred.size(); i++) {
      state_pred[i] += dstate_pred[i] * time_step;
      state_prey[i] += dstate_prey[i] * time_step;
    }

    // renormalize the quaternions
    gazebo::math::Quaternion ed(state_pred[6], state_pred[3], state_pred[4], state_pred[5]);
    gazebo::math::Quaternion ey(state_prey[6], state_prey[3], state_prey[4], state_prey[5]);
    ed.Normalize();
    ey.Normalize();
    state_pred[3] = ed.x;
    state_pred[4] = ed.y;
    state_pred[5] = ed.z;
    state_pred[6] = ed.w;
    state_prey[3] = ey.x;
    state_prey[4] = ey.y;
    state_prey[5] = ey.z;
    state_prey[6] = ey.w;

    // convert back to the state space
    pp_state_c qf( state_pred, state_prey );
    qf.write_ompl_state( statespace, result );

    statespace->enforceBounds( result );
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


#endif // _INTEGRATOR_H_

