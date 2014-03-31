#include "osthread.h"

#include "time.h"
#include "scheduler.h"
#include <sstream>

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

#include <stdio.h>
void osthread_c::dispatch( thread_p& current_thread ) {
  std::stringstream log_msg;

  thread_p this_thread = shared_from_this();

  if( this_thread != current_thread ) {

    log_msg << "dispatching " << this_thread->name << "\n";
    _log->write( log_msg.str() ); log_msg.str( std::string() ); log_msg.clear();
    // Note: may need to check thread type here
    boost::shared_ptr<osthread_c> current_osthread = boost::dynamic_pointer_cast<osthread_c>(current_thread);
    boost::shared_ptr<osthread_c> this_osthread = boost::dynamic_pointer_cast<osthread_c>(this_thread);

    if( current_osthread ) {
      current_osthread->lower_priority();
      log_msg << "current_osthread: " << current_osthread->name << ", _os_priority:" << current_osthread->_os_priority << "\n";
      _log->write( log_msg.str() ); log_msg.str( std::string() ); log_msg.clear();
    }
 
    // flush log here as the time accounting will not be attributed to clients
    if( _log->size() >= _log->capacity() / 2 )
      _log->flush();

    this_osthread->raise_priority();
    log_msg << "this_osthread: " << this_osthread->name << ", os_priority:" << this_osthread->_os_priority << "\n";
    _log->write( log_msg.str() ); log_msg.str( std::string() ); log_msg.clear();

    current_thread = this_thread;
  }
 
  //timestamp_t ts_select, ts_diff;

  log_msg << "selecting\n";
  _log->write( log_msg.str() ); log_msg.str( std::string() ); log_msg.clear();
  timestamp = generate_timestamp();
  select();
  progress += generate_timestamp() - timestamp;
  //ts_select = generate_timestamp();
  //ts_diff = ts_select - timestamp;
  //realtime_t rt_diff = timestamp_to_realtime( ts_diff, cpu_speed );
  // accounting for select time
  //progress += rt_diff;

  log_msg << "reading notifications\n";
  _log->write( log_msg.str() ); log_msg.str( std::string() ); log_msg.clear();
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
