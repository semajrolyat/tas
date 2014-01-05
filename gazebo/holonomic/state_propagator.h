#ifndef _STATE_PROPAGATOR_H_
#define _STATE_PROPAGATOR_H_

#include "common.h"
//#include "ship.h"
#include "integrator.h"

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/SpaceInformation.h>

//-----------------------------------------------------------------------------

class state_propagator_c : public ompl::control::StatePropagator {
public:
  integrator_c integrator;
  const ompl::base::StateSpace *space;
  ship_p pred;
  ship_p prey;

  //---------------------------------------------------------------------------
  state_propagator_c( const ompl::control::SpaceInformationPtr &si, ship_p _pred, ship_p _prey, ode_f _ode, prey_command_f _pc ) : 
    ompl::control::StatePropagator(si),
    integrator( si->getStateSpace().get(), 0.0, _ode, _pc ),
    space( si->getStateSpace().get() ),
    pred( _pred ),
    prey( _prey )
  { }

  //---------------------------------------------------------------------------
  virtual void propagate( const ompl::base::State* state, const ompl::control::Control* control, double duration, ompl::base::State* result ) const {
    integrator.propagate( state, control, result, pred, prey );
  }

  //---------------------------------------------------------------------------
  void setIntegrationTimeStep( const double& time_step ) {
    integrator.set_time_step( time_step );
  }

  //---------------------------------------------------------------------------
  double getIntegrationTimeStep( void ) const {
    return integrator.get_time_step();
  }
};

//-----------------------------------------------------------------------------

#endif // _STATE_PROPAGATOR_H_

