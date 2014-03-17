#include "osthread.h"

#include "time.h"
#include "scheduler.h"

// TODO: Add constructor that includes all the ancillary and potentially private
//       data
//-----------------------------------------------------------------------------
osthread_c::osthread_c( void ) {
  assert( scheduler_c::get_realtime_min_priority(_min_priority) == scheduler_c::ERROR_NONE );
  assert( scheduler_c::get_realtime_max_priority(_max_priority) == scheduler_c::ERROR_NONE );
  _priority_step = _max_priority - _min_priority;
}

//-----------------------------------------------------------------------------
osthread_c::osthread_c( select_f select, read_notifications_f read_notifications ) {
  this->select = select;
  this->read_notifications = read_notifications;

  assert( scheduler_c::get_realtime_min_priority(_min_priority) == scheduler_c::ERROR_NONE );
  assert( scheduler_c::get_realtime_max_priority(_max_priority) == scheduler_c::ERROR_NONE );
  _priority_step = _max_priority - _min_priority;
}

//-----------------------------------------------------------------------------
osthread_c::~osthread_c( void ) {

}

//-----------------------------------------------------------------------------
void osthread_c::dispatch( thread_p& current_thread ) {
  thread_p this_thread = shared_from_this();

  if( this_thread != current_thread ) {
    // Note: may need to check thread type here
    boost::shared_ptr<osthread_c> current = boost::dynamic_pointer_cast<osthread_c>(current_thread);
    boost::shared_ptr<osthread_c> ths = boost::dynamic_pointer_cast<osthread_c>(this_thread);

    if( current ) current->lower_priority();
    ths->raise_priority();
    current_thread = this_thread;
  }
  
  timestamp = generate_timestamp();
  select();
  // accounting for select time
  progress += generate_timestamp() - timestamp;
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
  error = scheduler_c::increase_realtime_priority( pid, _priority, _priority_step, _max_priority );
  if( error != scheduler_c::ERROR_NONE ) {
    // TODO: how to recover?
  }
}

//-----------------------------------------------------------------------------
void osthread_c::lower_priority( void ) {

  scheduler_c::error_e error;
  error = scheduler_c::decrease_realtime_priority( pid, _priority, _priority_step, _min_priority );
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
