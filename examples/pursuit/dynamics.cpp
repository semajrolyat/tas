#include "dynamics.h"

//-----------------------------------------------------------------------------
dynamics_c::dynamics_c( void ) {

}

//-----------------------------------------------------------------------------
dynamics_c::~dynamics_c( void ) {

}

//-----------------------------------------------------------------------------
void dynamics_c::dispatch( thread_p& current_thread ) {
  // TODO: step size needs to be more intelligent, i.e. parameterized
  const realtime_t STEP_SIZE = 0.001;

  // do dynamics
  step( STEP_SIZE );

  current_thread = shared_from_this();

  // TODO: expose plugin method to query dynamics_time;
  //current_thread->progress = dynamics_time();
}

//-----------------------------------------------------------------------------
void dynamics_c::terminate( void ) {

}

//-----------------------------------------------------------------------------
void dynamics_c::step( const realtime_t& dt ) {

}

//-----------------------------------------------------------------------------
