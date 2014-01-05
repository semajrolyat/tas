#ifndef _INTEGRATOR_H_
#define _INTEGRATOR_H_

#include "common.h"
#include "command.h"
#include "state.h"
#include "utilities.h"
//#include "ship.h"

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/control/Control.h>

//-----------------------------------------------------------------------------

// the integrator for the predator and the prey
class integrator_c {
private:
  ompl::base::StateSpace *statespace;
  double time_step;
  ode_f ode;
  prey_command_f prey_command;

  //---------------------------------------------------------------------------
public:
  integrator_c( ompl::base::StateSpace *_statespace, const double& _time_step, ode_f _ode, prey_command_f _pc ) : 
    statespace( _statespace ), 
    time_step( _time_step),
    ode( _ode ),
    prey_command( _pc )
  { }

  //---------------------------------------------------------------------------
  void propagate( const ompl::base::State *start, const ompl::control::Control *control, ompl::base::State *result, ship_p pred, ship_p prey ) const {
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
    prey_command(state_pred, state_prey, u_prey, pred, prey, 0);

    // get the ODEs for the predator and prey
    ode( state_pred, u_pred, dstate_pred, pred );
    ode( state_prey, u_prey, dstate_prey, prey );

    // update the predator and prey states
    for (unsigned i=0; i< state_pred.size(); i++) {
      state_pred[i] += dstate_pred[i] * time_step;
      state_prey[i] += dstate_prey[i] * time_step;
    }

    // renormalize the quaternions
    double md = 0, my = 0;
    for( unsigned i = 0; i < 4; i++ ) {
      md += state_pred[i] * state_pred[i];
      my += state_prey[i] * state_prey[i];
    }
    md = sqrt( md );
    my = sqrt( my );
    for( unsigned i = 0; i < 4; i++ ) {
      state_pred[i] /= md;
      state_prey[i] /= my;
    }

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

//-----------------------------------------------------------------------------

#endif // _INTEGRATOR_H_

