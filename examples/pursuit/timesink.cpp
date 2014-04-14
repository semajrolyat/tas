#include "timesink.h"
#include <limits>

#include <stdio.h>

//-----------------------------------------------------------------------------
timesink_c::timesink_c( const char* name ) {
  sprintf( this->name, "%s", name );
  this->policy = scheduler_c::PROGRESS;

  temporal_progress = 0;
  computational_progress = 0;
}
/*
//-----------------------------------------------------------------------------
timesink_c::timesink_c( const timesink_p& owner ) {
  this->policy = scheduler_c::PROGRESS;
  this->owner = owner;

  temporal_progress = 0;
  computational_progress = 0;
}
*/
//-----------------------------------------------------------------------------
timesink_c::timesink_c( const char* name, const timesink_p& owner, const scheduler_c::policy_e& policy = scheduler_c::PROGRESS ) {
  sprintf( this->name, "%s", name );
  this->policy = policy;
  this->owner = owner;

  temporal_progress = 0;
  computational_progress = 0;
}

//-----------------------------------------------------------------------------
timesink_c::timesink_c( const scheduler_c::policy_e& policy ) {
  this->policy = policy;

  temporal_progress = 0;
  computational_progress = 0;
}

//-----------------------------------------------------------------------------
timesink_c::~timesink_c( void ) {

}

//-----------------------------------------------------------------------------
void timesink_c::dispatch( thread_p& current_thread ) {
  char spstr[512];

  thread_p this_thread = shared_from_this();

  // set the parent's current_thread to this thread
  current_thread = this_thread;

  sprintf( spstr, "dispatching %s: computational_progress[%llu], temporal_progress[%llu]\n", name, computational_progress, temporal_progress );
  //_info->write( spstr );
  //printf( "%s", spstr );

  // update the system
  scheduler_c::step_system( this_thread, current_thread, run_queue, block_queue, info );

  // TODO: <Critical> If there was preemption, the child thread may not get
  // properly rescheduled.  Two obvious cases to consider 1) if the child
  // received a thread specific message (it will get rescheduled in process
  // notifications), 2) if the system received a message the child will not get
  // rescheduled.



  // Note: the following may not be fully correct as step_system has schedule
  // inside it.  TODO: determine if a correction is required
  // schedule the next thread
  thread_p next_thread = scheduler_c::schedule( this_thread, run_queue );

  if( next_thread ) {
    // if there is a runnable thread
    // next gives the time to execute next (may be the future past virtual
    // time of the simulator)
    temporal_progress = next_thread->temporal_progress;
    computational_progress = next_thread->computational_progress;
  } else {
    // all threads are blocked
    // if the cpu is idle, then our progress is defined by the time the least
    // progressed blocked thread wants to wakeup

    //temporal_progress = current_thread->temporal_progress;
    //computational_progress = current_thread->computational_progress;
    temporal_progress = 0;
    computational_progress = 0;

    for( unsigned i = 0; i < block_queue.size(); i++ ) {
      thread_p thread = block_queue.element(i);

      temporal_progress += thread->temporal_progress;
      computational_progress += thread->computational_progress;
    }

    

/*
    // set the progress to inordinately high number
    this->temporal_progress = std::numeric_limits<realtime_t>::infinity();
    // find the one with minimum progress to set the timesink's progress by
    // TODO the block queue is a heap now.  Should be a direct query rather than
    // search.  Refactor.
    for( unsigned i = 0; i < block_queue.size(); i++ ) {
      thread_p thread = block_queue.element(i);
      //this->progress = std::min( this->progress, thread->progress );

    }
*/
/*
    // timesink progress measured by top of the heap of blocking threads
    this->progress = block_queue.front()->progress;
*/
  }

  if( info ) {
    sprintf( spstr, "%s preempted(timesink.cpp): computational_progress[%llu], temporal_progress[%llu]\n", name, computational_progress, temporal_progress );
    info->write( spstr );
  }
}

//-----------------------------------------------------------------------------
void timesink_c::terminate( void ) {

}

//-----------------------------------------------------------------------------
