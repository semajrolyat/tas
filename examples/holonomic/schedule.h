#ifndef _SCHEDULE_H_
#define _SCHEDULE_H_

#include "tas.h"
#include "error.h"

#include <queue>

#include "thread.h"

//-----------------------------------------------------------------------------
//WHERE DOES THIS BELONG??
// Is it a function of the scheduler or is it more general.  Point is that both child threads
// and the main process will need to make use of this function
error_e set_cpu( const int& pid, const int& processor ) {
  // restrict the cpu set s.t. controller only runs on a single processor
  cpu_set_t cpuset_mask;
  // zero out the cpu set
  CPU_ZERO( &cpuset_mask );
  // set the cpu set s.t. controller only runs on 1 processor specified by processor
  CPU_SET( processor, &cpuset_mask );

  if( sched_setaffinity( pid, sizeof(cpuset_mask), &cpuset_mask ) == -1 ) {
    // there was an error setting the affinity for the coordinator
    // NOTE: can check errno if this occurs
    //sprintf( strbuffer, "(coordinator.cpp) set_schedule() failed calling sched_setaffinity(coordinator_pid,...)\n" );
    //error_log.write( strbuffer );
    return ERROR_FAILED;
  }

//  // testing sanity check ... TO BE COMMENTED
//  int ret = sched_getaffinity( 0, sizeof(cpuset_mask), &cpuset_mask );
//  printf( " sched_getaffinity = %d, cpuset_mask = %08lx\n", sizeof(cpuset_mask), cpuset_mask );

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------

// priority, processor
error_e set_realtime_schedule_max( const int& pid, int& priority ) {

  struct sched_param param;

  // set the scheduling policy and the priority where priority should be the
  // highest possible priority, i.e. min, for the round robin scheduler
  param.sched_priority = sched_get_priority_max( SCHED_RR );
  sched_setscheduler( pid, SCHED_RR, &param );

  // validate the scheduling policy
  int policy = sched_getscheduler( pid );
  if( policy != SCHED_RR ) {
    return ERROR_FAILED;
  }

  // validate the priority
  sched_getparam( pid, &param );
  priority = param.sched_priority;

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------

error_e set_realtime_schedule_rel( const int& pid, int& priority, const int& priority_offset ) {

  struct sched_param param;

  param.sched_priority = sched_get_priority_max( SCHED_RR ) - priority_offset;
  sched_setscheduler( pid, SCHED_RR, &param );

  // query the current policy
  int policy = sched_getscheduler( pid );
  if( policy == -1 ) {
    return ERROR_FAILED;
  }

  // validate the priority
  sched_getparam( pid, &param );
  priority = param.sched_priority;

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------

enum scheduler_error_e {
  SCHED_ERROR_NONE = 0,
  SCHED_ERROR_THREAD_GENERAL,
  SCHED_ERROR_THREAD_SPAWN,
  SCHED_ERROR_THREAD_SET_CPU,
  SCHED_ERROR_THREAD_SET_SCHEDULE,
  SCHED_ERROR_THREAD_SET_POLICY,
  SCHED_ERROR_THREAD_SET_PRIORITY,
  SCHED_ERROR_UNKNOWN,
  SCHED_ERROR_PARAM,
  SCHED_ERROR_THREAD_NOEXIST,
  SCHED_ERROR_PERMISSION,
  SCHED_ERROR_VALIDATION
};

//-----------------------------------------------------------------------------

class scheduler_c {
public:
  scheduler_c( void ) { }
  virtual ~scheduler_c( void ) { }

  std::queue<thread_c> runqueue;
  std::queue<thread_c> readyqueue;
  std::queue<thread_c> waitqueue;

  // scheduling facilities


  //---------------------------------------------------------------------------

  //yourtime_t dispatch(thread \in \{controller, simulator\}, yourtime_t max_duration, thd_status_t *status);
  realtime_t dispatch( thread_c& thread, const realtime_t& max_duration, thread_status_e& status ) {

  }

  //---------------------------------------------------------------------------

  // gets the virtual time from a controlled process (or the "clock time" on a real robot)
  realtime_t get_time( thread_c& thread ) {

  }

  //---------------------------------------------------------------------------

  /// Creates the controller process with a priority level one step below the coordinator.
  /// Note: an ERROR_NONE in the coordinator does not really guarantee that the controller
  /// started properly
  scheduler_error_e create( thread_c& thread, const std::string& program, const int& priority, const int& processor ) {

    thread.type = PROCESS;
    thread.program = program;
    thread.status = THREAD_NEW;

    thread.pid = fork( );

    // handle any fork error
    if( thread.pid < 0 ) {
      return SCHED_ERROR_THREAD_SPAWN;
    }

    // if this is still the parent process then get out.
    if( thread.pid > 0 ) return SCHED_ERROR_NONE;

    // start of the child process
    thread.pid = getpid( );

    if( set_cpu( thread.pid, processor ) != ERROR_NONE ) {
      return SCHED_ERROR_THREAD_SET_CPU;
    }

    // Note: may need to parameterize the relative priority for the current function's signature
    if( set_realtime_schedule_rel( thread.pid, thread.priority, priority ) != ERROR_NONE ) {
      return SCHED_ERROR_THREAD_SET_POLICY;
    }

    // execute the program
    // TODO : error handling
    execl( thread.program.c_str(), thread.program.c_str(), NULL );

    // Note: this will need to become execv and pass any parameters

    // exit on fail to exec.  Can only get here if execl fails.
    // Note this is exiting the child process not the parent
    _exit( 1 );

    return SCHED_ERROR_NONE;
  }

  //---------------------------------------------------------------------------

  scheduler_error_e create( thread_c& thread, const int& priority, thread_callback_fn callback ) {

    pthread_attr_t attributes;
    struct sched_param param;

    thread.type = THREAD;
    thread.status = THREAD_NEW;

    // establish attributes
    if( pthread_attr_init( &attributes ) != 0 )
      return SCHED_ERROR_THREAD_GENERAL;

    // set an explicit schedule for the thread
    if( pthread_attr_setinheritsched( &attributes, PTHREAD_EXPLICIT_SCHED )  != 0 )
      return SCHED_ERROR_THREAD_SET_SCHEDULE;

    // set schedule policy to round robin
    if( pthread_attr_setschedpolicy( &attributes, SCHED_RR ) != 0 )
      return SCHED_ERROR_THREAD_SET_POLICY;

    // set the priority
    param.sched_priority = priority;
    if( pthread_attr_setschedparam( &attributes, &param ) != 0 )
      return SCHED_ERROR_THREAD_SET_PRIORITY;

    // spawn the thread
    if( pthread_create( &thread.thread, &attributes, callback, NULL ) != 0 )
      return SCHED_ERROR_THREAD_SPAWN;

    // clean up attributes
    if( pthread_attr_destroy( &attributes ) != 0)
      return SCHED_ERROR_THREAD_GENERAL;

    return SCHED_ERROR_NONE;
  }

  //---------------------------------------------------------------------------

  void suspend( thread_c& thread ) {

    if( thread.type == PROCESS )
      kill( thread.pid, SIGSTOP );

    thread.status = THREAD_READY;
  }

  //---------------------------------------------------------------------------

  /// Continue the controller from the coordinator process
  void resume( thread_c& thread ) {

    thread.status = THREAD_RUNNING;

    if( thread.type == PROCESS )
      kill( thread.pid, SIGCONT );
  }

  //---------------------------------------------------------------------------

  void shutdown( thread_c& thread ) {

    thread.status = THREAD_TERMINATED;

    if( thread.type == PROCESS )
      kill( thread.pid, SIGTERM );
      //kill( thread.pid, SIGKILL );
  }

  //---------------------------------------------------------------------------

  // Note: the thread must already have been created to use this method
  scheduler_error_e adjust_priority( thread_c& thread, const int& priority ) {
    struct sched_param param;

    if( thread.type == PROCESS ) {
      // Query current priority for the process
      if( sched_getparam( thread.pid, &param ) == -1 ) {
        // querying failed
        switch( errno ) {
        case EINVAL:
          // The argument param does not make sense for the current scheduling policy
          return SCHED_ERROR_PARAM;
        case EPERM:
          // The calling process does not have appropriate privileges
          return SCHED_ERROR_PERMISSION;
        case ESRCH:
          // The process whose ID is pid could not be found
          return SCHED_ERROR_THREAD_NOEXIST;
        default:
          return SCHED_ERROR_UNKNOWN;
        }
      }

      // If the process priority is already the requested priority, do nothing.
      if( param.sched_priority == priority ) return SCHED_ERROR_NONE;

      // Adjust the priority
      param.sched_priority = priority;
      if( sched_setparam( thread.pid, &param ) == -1 ) {
        // system call to update policy failed
        switch( errno ) {
        case EINVAL:
          // The scheduling policy is not one of the recognized policies, param is NULL, or param does not make sense for the policy
          return SCHED_ERROR_PARAM;
        case EPERM:
          // The calling process does not have appropriate privileges
          return SCHED_ERROR_PERMISSION;
        case ESRCH:
          // The process whose ID is pid could not be found
          return SCHED_ERROR_THREAD_NOEXIST;
        default:
          return SCHED_ERROR_UNKNOWN;
        }
      }

      // Requery the policy to validate
      if( sched_getparam( thread.pid, &param ) == -1 ) {
        // requerying failed
        switch( errno ) {
        case EINVAL:
          // The argument param does not make sense for the current scheduling policy
          return SCHED_ERROR_PARAM;
        case EPERM:
          // The calling process does not have appropriate privileges
          return SCHED_ERROR_PERMISSION;
        case ESRCH:
          // The process whose ID is pid could not be found
          return SCHED_ERROR_THREAD_NOEXIST;
        default:
          return SCHED_ERROR_UNKNOWN;
        }
      }

      // Validate
      if( param.sched_priority != priority ) {
        // Validation failed : system did not update policy properly
        return SCHED_ERROR_VALIDATION;
      }
    } else if( thread.type == THREAD ) {
      int policy;
      int result;

      // Query current policy for the pthread
      result = pthread_getschedparam( thread.thread, &policy, &param );
      if( result != 0 ) {
        // querying failed probably due to thread not existing
        switch( result ) {
        case ESRCH:
          // No thread with the ID thread could be found
          return SCHED_ERROR_THREAD_NOEXIST;
        default:
          return SCHED_ERROR_UNKNOWN;
        }
      }

      // If the thread priority is already the requested priority, do nothing.
      if( param.sched_priority == priority )  return SCHED_ERROR_NONE;

      // Adjust the priority
      param.sched_priority = priority;
      result = pthread_setschedparam( thread.thread, policy, &param );
      if( result != 0 ) {
        // system call to update policy failed
        switch( result ) {
        case ESRCH:
          // No thread with the ID thread could be found
          return SCHED_ERROR_THREAD_NOEXIST;
        case EINVAL:
          // Policy is not a recognized policy. or param does not make sense for the policy
          return SCHED_ERROR_PARAM;
        case EPERM:
          // The caller does not have the appropriate privileges to set the specified scheduling policy and parameters
          return SCHED_ERROR_PERMISSION;
        case ENOTSUP:
          // Attempt was made to set the policy or scheduling parameters to an unsupported value
          break;
        default:
          return SCHED_ERROR_UNKNOWN;
        }
      }

      // Requery the policy to validate
      result = pthread_getschedparam( thread.thread, &policy, &param );
      if( result != 0 ) {
        // requerying failed
        switch( result ) {
        case ESRCH:
          // No thread with the ID thread could be found
          return SCHED_ERROR_THREAD_NOEXIST;
        default:
          return SCHED_ERROR_UNKNOWN;
        }
      }

      // Validate
      if( param.sched_priority != priority ) {
        // Validation failed : system did not update policy properly
        return SCHED_ERROR_VALIDATION;
      }
    }

    // Only after setting the priority has been validated then update
    thread.priority = priority;

    return SCHED_ERROR_NONE;
  }
};

//-----------------------------------------------------------------------------


#endif // _SCHEDULE_H_
