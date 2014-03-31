#include "timesink.h"
#include <limits>

//-----------------------------------------------------------------------------
timesink_c::timesink_c( void ) {
  this->policy = scheduler_c::PROGRESS;
  // TODO: determine when to do initial heapification, i.e. make_heap
  progress = 0;
}

//-----------------------------------------------------------------------------
timesink_c::timesink_c( const scheduler_c::policy_e& policy ) {
  this->policy = policy;
  //_log = log;
  progress = 0;
}

//-----------------------------------------------------------------------------
timesink_c::~timesink_c( void ) {

}

//-----------------------------------------------------------------------------
void timesink_c::dispatch( thread_p& current_thread ) {
  // set the parent's current_thread to this thread
  current_thread = shared_from_this();

  // update the system
  scheduler_c::step_system( this->current_thread, run_queue, block_queue, log );

  // Note: the following may not be fully correct as step_system has schedule
  // inside it.  TODO: determine if a correction is required
  // schedule the next thread
  thread_p next_thread = scheduler_c::schedule( run_queue );

  if( next_thread ) {
    // if there is a runnable thread
    // next gives the time to execute next (may be the future past virtual
    // time of the simulator)
    this->progress = next_thread->progress;
  } else {
    // all threads are blocked
    // if the cpu is idle, then our progress is defined by the time the least
    // progressed blocked thread wants to wakeup

//    /*
    // set the progress to inordinately high number
    this->progress = std::numeric_limits<realtime_t>::infinity();

    // find the one with minimum progress to set the timesink's progress by
    // TODO the block queue is a heap now.  Should be a direct query rather than
    // search.  Refactor.
    for( unsigned i = 0; i < block_queue.size(); i++ ) {
      thread_p thread = block_queue.element(i);
      this->progress = std::min( this->progress, thread->progress );
    }
//    */
/*
    // timesink progress measured by top of the heap of blocking threads
    this->progress = block_queue.front()->progress;
*/
  }
}

//-----------------------------------------------------------------------------
void timesink_c::terminate( void ) {

}

//-----------------------------------------------------------------------------
