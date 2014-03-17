#include "scheduler.h"

#include <sys/types.h>
#include <signal.h>
#include <errno.h>

#include "thread.h"
#include "notification.h"

#include <stdio.h>             // remove after all debugging printf's removed

//-----------------------------------------------------------------------------
/// API function to activate a thread waiting to run.
/// @param runqueue the heap of threads ready to run for a given timesink.
/// @return the current thread at the top of the heap.
thread_p scheduler_c::schedule( std::vector<thread_p>& runqueue ) {
  thread_p thread;

  if( !runqueue.empty() ) {
    std::pop_heap( runqueue.begin(), runqueue.end() );
    thread = runqueue.back();
    runqueue.pop_back();
  }
  return thread;
}

//-----------------------------------------------------------------------------
/// API function to process notifications delivered to a thread.
/// @param thread the current thread.
/// @param runqueue the heap of threads ready to run for a given timesink.
/// @param waitqueue the heap of threads pending on I/O for a given timesink.
// TODO: change nomenclature from message to notification
void scheduler_c::process_notifications( thread_p& thread, std::vector<thread_p>& runqueue, std::vector<thread_p>& waitqueue ) {

  // assert that the thread is valid and that the message queue is not empty
  if( !thread || thread->message_queue.empty() ) return;

  // dequeue the message from the queue
  notification_t msg = thread->message_queue.front();
  thread->message_queue.pop();

  if( msg.type == notification_t::IDLE && msg.blocktill > thread->progress ) {
    // find the thread in the runqueue and remove it.
    for( std::vector<thread_p>::iterator it = runqueue.begin(); it != runqueue.end(); it++ ) {
      thread_p thd = *it;
      if( thd == thread ) {
        runqueue.erase( it );
        // reheapify
        std::sort_heap( runqueue.begin(), runqueue.end() );
        break;
      }
    }
    // to be comprehensive, might be inclined to check waitqueue,
    // but logically, it cannot be in the waitqueue if it was running

    // update its progress (forcasting in this way will account for 'idle time')
    thread->progress = msg.blocktill;

    // add to the waitqueue
    waitqueue.push_back( thread );
    std::push_heap( waitqueue.begin(), waitqueue.end() );
  }
  // TODO - Logic for sending message to other threads if need be
}

//-----------------------------------------------------------------------------
/// API function to determine whether a thread is ready to transition from 
/// waiting to running.
/// @param runqueue the heap of threads ready to run for a given timesink.
/// @param waitqueue the heap of threads pending on I/O for a given timesink.
void scheduler_c::process_wakeups( std::vector<thread_p>& runqueue, std::vector<thread_p>& waitqueue ) {

  // get the highest progress of the runqueue
  realtime_t time = runqueue.front()->progress;

  // iterate over each of the threads in the waitqueue
  for( std::vector<thread_p>::iterator it = waitqueue.begin(); it != waitqueue.end(); it++ ) {
    thread_p thread = *it;

    // if the thread's 'progress' is less than or equal to the highest progress
    if( thread->progress <= time ) {

      // remove the thread from the waitqueue
      waitqueue.erase( it );

      // reheapify the waitqueue
      std::sort_heap( waitqueue.begin(), waitqueue.end() );

      // push the thread onto the runqueue heap
      runqueue.push_back( thread );
      std::push_heap( runqueue.begin(), runqueue.end() );
    }
  }
}

//-----------------------------------------------------------------------------
/// API function to advance the state of the system.
/// @param current_thread the active thread for a given timesink.
/// @param runqueue the heap of threads ready to run for a given timesink.
/// @param waitqueue the heap of threads pending on I/O for a given timesink.
void scheduler_c::step_system( thread_p& current_thread, std::vector<thread_p>& runqueue, std::vector<thread_p>& waitqueue ) {
  // schedule the next thread
  thread_p next_thread = schedule( runqueue );

  // if there is a next thread, dispatch it
  if( next_thread ) { 
    printf( "next_thread:%s\n", next_thread->name.c_str() );

    next_thread->dispatch( current_thread );
  
    // process any messages delivered to the current (next) thread
    process_notifications( current_thread, runqueue, waitqueue );
  }

  // process the waitqueue to see if any thread needs to be moved to runqueue
  process_wakeups( runqueue, waitqueue );
}

//-----------------------------------------------------------------------------
/// Forks the main process and executes the specified program in the newly
/// spawned child process after binding it to a cpu and scheduling it with a 
/// realtime policy.
/// @param pid the process identifier assigned to the newly forked process
/// @param priority_offset the offset from max priority to assign the process
/// @param processor the processor to bind the process to
/// @param program the name of the program to execute
/// @param args the argument list to pass to the program
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::create( pid_t& pid, const int& priority_offset, const cpu_id_t& cpu, const std::string& program, const std::string& args ) {
// TODO: adjust sig to use the args as we will use
// TODO: per discussion, any args should be written to a file or stdin then
//       passed into the new processes either by filename or dup stdin fd

  int priority;

  // fork the process
  pid = fork();

  // handle any fork error
  if( pid < 0 ) 
    return ERROR_CREATE;

  // if still inside the parent process, get out
  if( pid > 0 )
    return ERROR_NONE;

  // child process
  pid = getpid();

  // bind the child to the cpu  
  if( cpu_c::bind( pid, cpu ) != ERROR_NONE )
    _exit( 1 );     // kill the child process

  // set the scheduling policy and priority
  if( set_realtime_policy( pid, priority, priority_offset ) != ERROR_NONE )
    _exit( 1 );     // kill the child process

  // TODO: adjust the args execute the external program using either
  // execve( program.c_str(), ?, ? );
  // exec( program.c_str(), program.c_str(), NULL );

  // kill the child process
  _exit( 1 );

  // just a formality to eliminate the compiler warning
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// API function to send a suspend signal to a child process.  
/// Suspend functionality in the API is actually a function of priority and not
/// the result of an explicit signal, so this function is provided but not
/// necessarily used.
/// @param pid process identifier.
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::suspend( const pid_t& pid ) {
  assert( KILL_ALLOWED );

  int result = kill( pid, SIGSTOP );
  if( result == -1 ) {
    if( errno == EINVAL ) return ERROR_SIGNAL;
    else if( errno == EPERM ) return ERROR_PERMISSIONS;
    else if( errno == ESRCH ) return ERROR_NOEXIST;
    return ERROR_GENERAL;
  }
  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// API function to send a resume signal to a child process.
/// Resume functionality in the API is actually a function of priority and not
/// the result of an explicit signal, so this function is provided but not
/// necessarily used.
/// @param pid process identifier.
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::resume( const pid_t& pid ) {
  assert( KILL_ALLOWED );

  int result = kill( pid, SIGCONT );
  if( result == -1 ) {
    if( errno == EINVAL ) return ERROR_SIGNAL;
    else if( errno == EPERM ) return ERROR_PERMISSIONS;
    else if( errno == ESRCH ) return ERROR_NOEXIST;
    return ERROR_GENERAL;
  }
  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// API function to send a terminate signal to a child process.
/// @param pid process identifier.
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::terminate( const pid_t& pid ) {
  assert( KILL_ALLOWED );

  int result = kill( pid, SIGTERM );
  if( result == -1 ) {
    if( errno == EINVAL ) return ERROR_SIGNAL;
    else if( errno == EPERM ) return ERROR_PERMISSIONS;
    else if( errno == ESRCH ) return ERROR_NOEXIST;
    return ERROR_GENERAL;
  }
  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Sets the system schedule policy to realtime for a process with maximum 
/// priority.
/// @param pid a process identifier.
/// @param priority the priority that was assigned to the pid upon success.
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::set_realtime_policy( const pid_t& pid, int& priority ) {

  struct sched_param param;
  int max_priority;

  // get the maximum priority allowed by the scheduler
  error_e result = get_realtime_max_priority( max_priority );
  if( result != ERROR_NONE )
    return result;

  // set the scheduling parameter priority
  param.sched_priority = max_priority;

  // set the scheduler as with policy of round robin (realtime)
  if( sched_setscheduler( pid, SCHED_RR, &param ) == -1 ) {
    if( errno == EINVAL ) return ERROR_SIGNAL;
    else if( errno == EPERM ) return ERROR_PERMISSIONS;
    else if( errno == ESRCH ) return ERROR_NOEXIST;
    return ERROR_GENERAL;
  }

  // validate the scheduling policy
  int policy = sched_getscheduler( pid );
  if( policy == -1 ) {
    if( errno == EINVAL ) return ERROR_SIGNAL;
    else if( errno == EPERM ) return ERROR_PERMISSIONS;
    else if( errno == ESRCH ) return ERROR_NOEXIST;
    return ERROR_GENERAL;
  }
  if( policy != SCHED_RR ) return ERROR_POLICY;

  // query the priority again to check the value
  if( sched_getparam( pid, &param ) == -1 )
    return ERROR_PARAM_QUERY;

  // update the actual priority
  // Note: on ERROR_PRIORITY_VALIDATE, can look at current priority
  priority = param.sched_priority;

  // validate that the scheduler priority is equal to the max priority
  if( priority != max_priority ) 
    return ERROR_PRIORITY_VALIDATE;

  // success
  return ERROR_NONE;
}
//-----------------------------------------------------------------------------
/// Sets the system schedule policy to realtime for a process with a priority 
/// relative to the maximum priority value.
/// @param pid a process identifier.
/// @param priority the priority that was assigned to the pid upon success.
/// @param offset the offset of priority relative to the max priority.
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::set_realtime_policy( const pid_t& pid, int& priority, const int& offset ) {

  struct sched_param param;
  int max_priority, expected_priority;

  // get the maximum priority allowed by the scheduler
  error_e result = get_realtime_max_priority( max_priority );
  if( result != ERROR_NONE )
    return result;

  // compute the expected priority
  expected_priority = max_priority - offset;

  // set the scheduling parameter priority
  param.sched_priority = expected_priority;

  // set the scheduler as with policy of round robin (realtime)
  if( sched_setscheduler( pid, SCHED_RR, &param ) == -1 ) {
    if( errno == EINVAL ) return ERROR_SIGNAL;
    else if( errno == EPERM ) return ERROR_PERMISSIONS;
    else if( errno == ESRCH ) return ERROR_NOEXIST;
    return ERROR_GENERAL;
  }

  // validate the scheduling policy
  int policy = sched_getscheduler( pid );
  if( policy == -1 ) {
    if( errno == EINVAL ) return ERROR_SIGNAL;
    else if( errno == EPERM ) return ERROR_PERMISSIONS;
    else if( errno == ESRCH ) return ERROR_NOEXIST;
    return ERROR_GENERAL;
  }
  if( policy != SCHED_RR ) return ERROR_POLICY;

  // query the priority again to check the value
  if( sched_getparam( pid, &param ) == -1 )
    return ERROR_PARAM_QUERY;

  // update the actual priority
  // Note: on ERROR_PRIORITY_VALIDATE, can look at current priority
  priority = param.sched_priority;

  // validate that the scheduler priority is equal to the max priority
  if( priority != expected_priority ) 
    return ERROR_PRIORITY_VALIDATE;

  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Gets the minimum system schedule realtime priority for a process.
/// @param priority the minimum system priority.
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::get_realtime_min_priority( int& priority ) {

  // query the max priority
  priority = sched_get_priority_min( SCHED_RR );

  // on error
  // Note: must check return value, otherwise priority may be invalid
  if( priority == -1 ) return ERROR_PRIORITY_QUERY;

  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Gets the maximum system schedule realtime priority for a process.
/// @param priority the maximum system priority.
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::get_realtime_max_priority( int& priority ) {

  // query the max priority
  priority = sched_get_priority_max( SCHED_RR );

  // on error
  // Note: must check return value, otherwise priority may be invalid
  if( priority == -1 ) return ERROR_PRIORITY_QUERY;

  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Gets the system schedule priority for a process.
/// @param pid a process identifier.
/// @param priority the priority that is currently assigned to the process.
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::get_priority( const pid_t& pid, int& priority ) {
  sched_param param;

  if( sched_getparam( pid, &param ) == -1 ) 
    return ERROR_PARAM_QUERY;

  priority = param.sched_priority;
  // success
  return ERROR_NONE;
}
//-----------------------------------------------------------------------------
/// Sets the system schedule priority for a process.
/// @param pid a process identifier.
/// @param priority the input requested priority and the output actual priority.
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::set_priority( const pid_t& pid, int& priority ) {
  sched_param param;
  int requested_priority = priority;

  // get the scheduling parameter  
  if( sched_getparam( pid, &param ) == -1 )
    return ERROR_PARAM_QUERY;

  // if the scheduler priority is already set to the in priority, quasi success
  if( param.sched_priority == requested_priority ) 
    return ERROR_NONE;

  // set the scheduled priority to the requested priority
  param.sched_priority = requested_priority;
  if( sched_setparam( pid, &param ) == -1 )
    return ERROR_PARAM_UPDATE;

  // query the priority again to check the value
  if( sched_getparam( pid, &param ) == -1 )
    return ERROR_PARAM_QUERY;

  // update the actual priority
  // Note: on ERROR_PRIORITY_VALIDATE, can look at current priority
  priority = param.sched_priority;

  // validate that the scheduler priority is equal to the requested priority
  if( priority != requested_priority ) 
    return ERROR_PRIORITY_VALIDATE;
  
  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Decreases the system schedule realtime priority for a process by a 
/// specified amount.
/// @param pid a process identifier.
/// @param priority the priority that was assigned to the pid upon success.
/// @param offset the amount to decrease the currently assigned priority by.
/// @param min_priority the arbitrarily constrained floor of priorities which
/// must be greater than or equal to the system constrained floor of priorities.
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::decrease_realtime_priority( const pid_t& pid, int& priority, const int& offset, const int& min_priority ) {
  struct sched_param param;
  int min_allowed;
  int target_priority;
  error_e result;

  // SANITY CHECK that a negative number didn't creep in
  assert( offset > 0 );

  // get the minimum priority allowed by the scheduler
  result = get_realtime_min_priority( min_allowed );
  if( result != ERROR_NONE )
    return result;

  // if min specified is not constrained to the scheduler min then fail
  if( min_priority < min_allowed ) 
    return ERROR_PRIORITY_BOUNDS;

  // get the scheduling parameter  
  if( sched_getparam( pid, &param ) == -1 )
    return ERROR_PARAM_QUERY;

  target_priority = param.sched_priority - offset;
  // if the adjusted priority is less than the constrained min then fail
  if( target_priority < min_priority ) 
    return ERROR_PRIORITY_BOUNDS;

  // set the priority parameter
  param.sched_priority = target_priority;
  if( sched_setparam( pid, &param ) == -1 )
    return ERROR_PARAM_UPDATE;
  
  // query the priority again to check the value
  if( sched_getparam( pid, &param ) == -1 )
    return ERROR_PARAM_QUERY;

  // update the actual priority
  // Note: on ERROR_PRIORITY_VALIDATE, can look at current priority
  priority = param.sched_priority;

  // validate that the scheduler priority is now equal to the requested change
  if( param.sched_priority != target_priority )
    return ERROR_PRIORITY_VALIDATE;

  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Increases the system schedule realtime priority for a process by a 
/// specified amount.
/// @param pid a process identifier.
/// @param priority the priority that was assigned to the pid upon success.
/// @param offset the amount to increase the currently assigned priority by.
/// @param max_priority the arbitrarily constrained ceiling of priorities which
/// must be less than or equal to the system constrained ceiling of priorities.
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::increase_realtime_priority( const pid_t& pid, int& priority, const int& offset, const int& max_priority ) {
  struct sched_param param;
  int max_allowed;
  int target_priority;
  error_e result;

  // SANITY CHECK that a negative number didn't creep in
  assert( offset > 0 );

  // get the maximum priority allowed by the scheduler
  result = get_realtime_max_priority( max_allowed );
  if( result != ERROR_NONE )
    return result;

  // if max specified is not constrained to the scheduler max then fail
  if( max_priority > max_allowed ) 
    return ERROR_PRIORITY_BOUNDS;

  // get the scheduling parameter  
  if( sched_getparam( pid, &param ) == -1 )
    return ERROR_PARAM_QUERY;

  target_priority = param.sched_priority + offset;
  // if the adjusted priority exceeds the constrained max priority then fail
  if( target_priority > max_priority ) 
    return ERROR_PRIORITY_BOUNDS;

  // set the priority parameter
  param.sched_priority = target_priority;
  if( sched_setparam( pid, &param ) == -1 )
    return ERROR_PARAM_UPDATE;
  
  // query the priority again to check the value
  if( sched_getparam( pid, &param ) == -1 )
    return ERROR_PARAM_QUERY;

  // update the actual priority
  // Note: on ERROR_PRIORITY_VALIDATE, can look at current priority
  priority = param.sched_priority;

  // validate that the scheduler priority is now equal to the requested change
  if( param.sched_priority != target_priority )
    return ERROR_PRIORITY_VALIDATE;

  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Creates a pthread and schedules it under a realtime policy.
/// @param thread the reference to the pthread that is created.
/// @param priority the desired priority of the pthread.
/// @param worker the function to be assigned to the thread.
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::create( pthread_t& thread, const int& priority, worker_f worker ) {
  pthread_attr_t atts;
  struct sched_param param;
 
  // establish attributes
  if( pthread_attr_init( &atts ) != 0 )
    return ERROR_GENERAL; 

  // set an explicit schedule for the thread
  if( pthread_attr_setinheritsched( &atts, PTHREAD_EXPLICIT_SCHED )  != 0 ) 
    return ERROR_SCHEDULE; 

  // set schedule policy to round robin
  if( pthread_attr_setschedpolicy( &atts, SCHED_RR ) != 0 )
    return ERROR_POLICY;
 
  // set the priority
  param.sched_priority = priority;
  if( pthread_attr_setschedparam( &atts, &param ) != 0 )
    return ERROR_PARAM_UPDATE;

  // spawn the thread
  if( pthread_create( &thread, &atts, worker, NULL ) != 0 ) 
    return ERROR_CREATE;

  // clean up attributes
  if( pthread_attr_destroy( &atts ) != 0 )
    return ERROR_GENERAL; 

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Gets the system schedule priority for a pthread.
/// @param thread the pthread.
/// @param priority the priority that is currently assigned to the process.
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::get_priority( const pthread_t& thread, int& priority ) {

  sched_param param;
  int policy;

  int result = pthread_getschedparam( thread, &policy, &param );
  if( result != 0 )
    return ERROR_PARAM_QUERY;

  priority = param.sched_priority;
  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Sets the system schedule priority for a pthread.
/// @param thread the pthread.
/// @param priority the input requested priority and the output actual priority.
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::set_priority( const pthread_t& thread, int& priority ) {

  sched_param param;
  int policy;
  int requested_priority = priority;
  int result;

  // get the scheduling parameter    
  result = pthread_getschedparam( thread, &policy, &param );
  if( result != 0 )
    return ERROR_PARAM_QUERY;

  // if the scheduler priority is already set to the in priority, quasi success
  if( param.sched_priority == requested_priority )
    return ERROR_NONE;

  // set the scheduled priority to the requested priority
  param.sched_priority = requested_priority;
  result = pthread_setschedparam( thread, SCHED_RR, &param ); 
  if( result != 0 )
    return ERROR_PARAM_UPDATE;

  // query the priority again to check the value
  result = pthread_getschedparam( thread, &policy, &param );
  if( result != 0 )
    return ERROR_PARAM_QUERY;

  // update the actual priority
  // Note: on ERROR_PRIORITY_VALIDATE, can look at current priority
  priority = param.sched_priority;

  // validate that the scheduler priority is equal to the requested priority
  if( priority != requested_priority ) 
    return ERROR_PRIORITY_VALIDATE;
 
  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Decreases the system schedule realtime priority for a pthread by a 
/// specified amount.
/// @param thread the pthread.
/// @param priority the priority that was assigned to the pid upon success.
/// @param offset the amount to decrease the currently assigned priority by.
/// @param min_priority the arbitrarily constrained floor of priorities which
/// must be greater than or equal to the system constrained floor of priorities.
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::decrease_realtime_priority( const pthread_t& thread, int& priority, const int& offset, const int& min_priority ) {
  struct sched_param param;
  int min_allowed;
  int target_priority;
  error_e result;
  int presult;
  int policy;

  // SANITY CHECK that a negative number didn't creep in
  assert( offset > 0 );

  // get the minimum priority allowed by the scheduler
  result = get_realtime_min_priority( min_allowed );
  if( result != ERROR_NONE )
    return result;

  // if min specified is not constrained to the scheduler min then fail
  if( min_priority < min_allowed ) 
    return ERROR_PRIORITY_BOUNDS;

  // get the scheduling parameter    
  presult = pthread_getschedparam( thread, &policy, &param );
  if( presult != 0 )
    return ERROR_PARAM_QUERY;

  // Note: might be worthwhile to check the policy here

  target_priority = param.sched_priority - offset;
  // if the adjusted priority is less than the constrained min then fail
  if( target_priority < min_priority ) 
    return ERROR_PRIORITY_BOUNDS;

  // set the priority parameter
  param.sched_priority = target_priority;
  presult = pthread_setschedparam( thread, SCHED_RR, &param ); 
  if( presult != 0 )
    return ERROR_PARAM_UPDATE;
  
  // query the priority again to check the value
  presult = pthread_getschedparam( thread, &policy, &param );
  if( presult != 0 )
    return ERROR_PARAM_QUERY;

  // Note: might be worthwhile to check the policy again here

  // update the actual priority
  // Note: on ERROR_PRIORITY_VALIDATE, can look at current priority
  priority = param.sched_priority;

  // validate that the scheduler priority is now equal to the requested change
  if( param.sched_priority != target_priority )
    return ERROR_PRIORITY_VALIDATE;

  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Increases the system schedule realtime priority for a pthread by a 
/// specified amount.
/// @param thread the pthread.
/// @param priority the priority that was assigned to the pid upon success.
/// @param offset the amount to increase the currently assigned priority by.
/// @param max_priority the arbitrarily constrained ceiling of priorities which
/// must be less than or equal to the system constrained ceiling of priorities.
/// @return indicator of operation success or specified error.
scheduler_c::error_e scheduler_c::increase_realtime_priority( const pthread_t& thread, int& priority, const int& offset, const int& max_priority ) {
  struct sched_param param;
  int max_allowed;
  int target_priority;
  error_e result;
  int presult;
  int policy;

  // SANITY CHECK that a negative number didn't creep in
  assert( offset > 0 );

  // get the maximum priority allowed by the scheduler
  result = get_realtime_max_priority( max_allowed );
  if( result != ERROR_NONE )
    return result;

  // if max specified is not constrained to the scheduler max then fail
  if( max_priority > max_allowed ) 
    return ERROR_PRIORITY_BOUNDS;

  // get the scheduling parameter  
  presult = pthread_getschedparam( thread, &policy, &param );
  if( presult != 0 )
    return ERROR_PARAM_QUERY;

  // Note: might be worthwhile to check the policy here

  target_priority = param.sched_priority + offset;
  // if the adjusted priority exceeds the constrained max priority then fail
  if( target_priority > max_priority ) 
    return ERROR_PRIORITY_BOUNDS;

  // set the priority parameter
  param.sched_priority = target_priority;
  presult = pthread_setschedparam( thread, SCHED_RR, &param ); 
  if( presult != 0 )
    return ERROR_PARAM_UPDATE;
  
  // query the priority again to check the value
  presult = pthread_getschedparam( thread, &policy, &param );
  if( presult != 0 )
    return ERROR_PARAM_QUERY;

  // Note: might be worthwhile to check the policy again here

  // update the actual priority
  // Note: on ERROR_PRIORITY_VALIDATE, can look at current priority
  priority = param.sched_priority;

  // validate that the scheduler priority is now equal to the requested change
  if( param.sched_priority != target_priority )
    return ERROR_PRIORITY_VALIDATE;

  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
