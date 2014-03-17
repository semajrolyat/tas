#include "timesink.h"
#include <limits>

//-----------------------------------------------------------------------------
timesink_c::timesink_c( void ) {
  this->policy = scheduler_c::PROGRESS;
  // TODO: determine when to do initial heapification, i.e. make_heap
}

//-----------------------------------------------------------------------------
timesink_c::timesink_c( const scheduler_c::policy_e& policy ) {
  this->policy = policy;
}

//-----------------------------------------------------------------------------
timesink_c::~timesink_c( void ) {

}

//-----------------------------------------------------------------------------
void timesink_c::dispatch( thread_p& current_thread ) {
  // set the parent's current_thread to this thread
  current_thread = shared_from_this();

  // update the system
  scheduler_c::step_system( this->current_thread, runqueue, waitqueue );

  // Note: the following may not be fully correct as step_system has schedule
  // inside it.  TODO: determine if a correction is required
  // schedule the next thread
  thread_p next = scheduler_c::schedule( runqueue );

  if( next ) {
    // if there is a runnable thread
    // next gives the time to execute next (may be the future past virtual
    // time of the simulator)
    this->progress = next->progress;
  } else {
    // all threads are blocked
    // if the cpu is idle, then our progress is defined by the time the least
    // progressed blocked thread wants to wakeup

    this->progress = std::numeric_limits<realtime_t>::infinity();
    for( std::vector<thread_p>::iterator it = waitqueue.begin(); it != waitqueue.end(); it++ ) {
      thread_p thread = *it;
      this->progress = std::min( this->progress, thread->progress );
    }    
  }
}

//-----------------------------------------------------------------------------
void timesink_c::terminate( void ) {

}

//-----------------------------------------------------------------------------
