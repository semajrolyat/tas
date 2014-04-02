#include "osthread.h"

#include "time.h"
#include "scheduler.h"
//#include <sstream>
#include <stdio.h>

// TODO: Add constructor that includes all the ancillary and potentially private
//       data
//-----------------------------------------------------------------------------
osthread_c::osthread_c( void ) {
  assert( scheduler_c::get_realtime_min_priority(_min_os_priority) == scheduler_c::ERROR_NONE );
  assert( scheduler_c::get_realtime_max_priority(_max_os_priority) == scheduler_c::ERROR_NONE );
  _os_priority_step = _max_os_priority - _min_os_priority;

  run_time = 0.0;
  wait_time = 0.0;
  block_time = 0.0;
  progress = 0;
}

//-----------------------------------------------------------------------------
osthread_c::osthread_c( select_f select, read_notifications_f read_notifications, log_c* log ) {
  this->select = select;
  this->read_notifications = read_notifications;
  _log = log;

  assert( scheduler_c::get_realtime_min_priority(_min_os_priority) == scheduler_c::ERROR_NONE );
  assert( scheduler_c::get_realtime_max_priority(_max_os_priority) == scheduler_c::ERROR_NONE );
  _os_priority_step = _max_os_priority - _min_os_priority;

  run_time = 0.0;
  wait_time = 0.0;
  block_time = 0.0;
  progress = 0;
}

//-----------------------------------------------------------------------------
osthread_c::~osthread_c( void ) {

}

//-----------------------------------------------------------------------------
void osthread_c::dispatch( thread_p& current_thread ) {
  char spstr[512];
  osthread_p current_osthread, this_osthread;
  thread_p this_thread;

  // get a reference to the thread about to be dispatched
  this_thread = shared_from_this();
  // get an osthread pointer to the thread about to be dispatched
  this_osthread = boost::dynamic_pointer_cast<osthread_c>(this_thread);

  //--------
  // check if current thread has a valid reference (otherwise was nothing)
  if( current_thread ) {
    // sanity check
    assert( current_thread->type() == thread_c::OSTHREAD );

    // get an osthread pointer to the thread previously dispatched
    current_osthread = boost::dynamic_pointer_cast<osthread_c>(current_thread);

    // lower the priority of the current thread
    current_osthread->lower_priority();

    // log information about the switch
    if( current_thread != this_thread ) {
      sprintf( spstr, "switching between current[%s] and next[%s] threads\n", current_thread->name.c_str(), this_thread->name.c_str() );
      _log->write( spstr );
      printf( "%s", spstr );
    }
  }

  // flush log here as the time accounting will not be attributed to clients 
  if( _log->size() >= _log->capacity() / 2 )
    _log->flush();

  // raise the priority of the thread being dispatched
  this_osthread->raise_priority();

  sprintf( spstr, "dispatching %s at priority %d\n", this_osthread->name.c_str(), this_osthread->_os_priority );
  _log->write( spstr );
  printf( "%s", spstr );

  // set the current thread as the one being dispatched
  current_thread = this_thread;

  //--------
  sprintf( spstr, "selecting\n" );
  _log->write( spstr );
  printf( "%s", spstr );

  timestamp = generate_timestamp();
  select();
  progress += generate_timestamp() - timestamp;

  //--------
  sprintf( spstr, "reading notifications\n" );
  _log->write( spstr );
  printf( "%s", spstr );

  read_notifications();
}
/*
//-----------------------------------------------------------------------------
void osthread_c::terminate( void ) {
  assert( true );     // for now we don't want to allow this
  // TODO: add a notification based means to terminate
}
*/
//-----------------------------------------------------------------------------
void osthread_c::raise_priority( void ) {

  scheduler_c::error_e error;
  error = scheduler_c::increase_realtime_priority( pid, _os_priority, _os_priority_step, _max_os_priority );
  if( error != scheduler_c::ERROR_NONE ) {
    // TODO: how to recover?
  }
}

//-----------------------------------------------------------------------------
void osthread_c::lower_priority( void ) {

  scheduler_c::error_e error;
  error = scheduler_c::decrease_realtime_priority( pid, _os_priority, _os_priority_step, _min_os_priority );
  if( error != scheduler_c::ERROR_NONE ) {
    // TODO: how to recover?
  }
}
/*
//-----------------------------------------------------------------------------
void osthread_c::block( void ) {
  kill( pid, SIGSTOP );
}

//-----------------------------------------------------------------------------
void osthread_c::unblock( void ) {
  kill( pid, SIGCONT );
}
*/
//-----------------------------------------------------------------------------
