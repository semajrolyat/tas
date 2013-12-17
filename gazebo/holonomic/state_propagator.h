#ifndef _GAZEBO_SHIP_STATE_PROPAGATOR_H_
#define _GAZEBO_SHIP_STATE_PROPAGATOR_H_

#include "ship.h"
#include "integrator.h"

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/SpaceInformation.h>

//-----------------------------------------------------------------------------
class state_propagator_c : public ompl::control::StatePropagator {
public:
  euler_integrator_c< ship_c > integrator;
  const ompl::base::StateSpace *space;
  ship_c* ship;

  //---------------------------------------------------------------------------
  state_propagator_c( const ompl::control::SpaceInformationPtr &si, ship_c* _ship) : 
    ompl::control::StatePropagator(si),
    integrator( si->getStateSpace().get(), 0.0 ),
    space( si->getStateSpace().get() ),
    ship(_ship)
  { }

  //---------------------------------------------------------------------------
  virtual void propagate( const ompl::base::State* state, const ompl::control::Control* control, const double duration, ompl::base::State* result ) const {
    integrator.propagate( state, control, duration, result, ship );
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

class pp_state_propagator_c : public ompl::control::StatePropagator {
public:
  pp_integrator_c< ship_c > integrator;
  const ompl::base::StateSpace *space;
  ship_c* pred;
  ship_c* prey;

  //---------------------------------------------------------------------------
  pp_state_propagator_c( const ompl::control::SpaceInformationPtr &si, ship_c* _pred, ship_c* _prey) : 
    ompl::control::StatePropagator(si),
    integrator( si->getStateSpace().get(), 0.0 ),
    space( si->getStateSpace().get() ),
    pred(_pred),
    prey(_prey)
  { }

  //---------------------------------------------------------------------------
  virtual void propagate( const ompl::base::State* state, const ompl::control::Control* control, const double duration, ompl::base::State* result ) const {
    integrator.propagate( state, control, duration, result, pred, prey );
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

#endif // _GAZEBO_SHIP_STATE_PROPAGATOR_H_

