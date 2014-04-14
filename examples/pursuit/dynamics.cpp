#include "dynamics.h"

#include <stdio.h>

/*
//-----------------------------------------------------------------------------
dynamics_c::dynamics_c( void ) {
  
}
*/
//-----------------------------------------------------------------------------
dynamics_c::dynamics_c( const char* name, const timesink_p& owner, const cpu_speed_t& cpu_speed ) : 
  timesink_c( name, owner, scheduler_c::PROGRESS )
{
//  sprintf( this->name, "%s", name );
  this->owner = owner;

  temporal_progress = 0;
  computational_progress = 0;

  _cpu_speed = cpu_speed;
  _step_size_cycles = seconds_to_cycles( STEP_SIZE_SECONDS, cpu_speed );
}

//-----------------------------------------------------------------------------
dynamics_c::~dynamics_c( void ) {

}

//-----------------------------------------------------------------------------
void dynamics_c::dispatch( thread_p& current_thread ) {
  char spstr[512];

  current_thread = shared_from_this();

  // TODO: step size needs to be more intelligent, i.e. parameterized
  //const realtime_t STEP_SIZE = 0.001;

  // do dynamics
  //step( STEP_SIZE );

  if( info ) {
    sprintf( spstr, "dispatching dynamics: computational_progress[%llu], temporal_progress[%llu], _step_size_cycles[%llu]\n", temporal_progress, computational_progress, _step_size_cycles );
    info->write( spstr );
  }

  step( _step_size_cycles );

  // TODO: expose plugin method to query dynamics_time;
  //current_thread->progress = dynamics_time();

  if( info ) {
    sprintf( spstr, "dynamics idle: computational_progress[%llu], temporal_progress[%llu], _step_size_cycles[%llu]\n", temporal_progress, computational_progress, _step_size_cycles );
    info->write( spstr );
  }

}

//-----------------------------------------------------------------------------
void dynamics_c::terminate( void ) {

}

/*
//-----------------------------------------------------------------------------
void dynamics_c::step( const realtime_t& dt ) {
  
}
*/
//-----------------------------------------------------------------------------
void dynamics_c::step( const cycle_t& dt ) {
  temporal_progress += dt;
  computational_progress += dt; 
}

//-----------------------------------------------------------------------------
