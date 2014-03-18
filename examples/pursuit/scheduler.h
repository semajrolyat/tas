/*-----------------------------------------------------------------------------
scheduler.h encapsulates all scheduling functionality for the system at both 
API level threads and POSIX level realtime scheduling.

author: James Taylor : jrt@gwu.edu
-----------------------------------------------------------------------------*/

#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

//-----------------------------------------------------------------------------

#include <vector>
#include <string>
#include <pthread.h>

#include "types.h"
#include "time.h"
#include "cpu.h"

//-----------------------------------------------------------------------------

#define KILL_ALLOWED false

//-----------------------------------------------------------------------------

/// Worker function pointer type for use with pthreads.
typedef void* (worker_f)( void* );

//-----------------------------------------------------------------------------

class scheduler_c {
public:

  /// The set of scheduling operation errors.
  enum error_e {
    ERROR_NONE = 0,          ///< Operation completed successfully.
    ERROR_SIGNAL,            ///< An invalid signal was sent.
    ERROR_PERMISSIONS,       ///< A process does not have required permissions.
    ERROR_NOEXIST,           ///< The process specified does not exist.
    ERROR_CREATE,            ///< A failure to create the thread or process.
    ERROR_SCHEDULE,          ///< A failure to set schedule.
    ERROR_POLICY,            ///< A failure to set realtime policy.
    ERROR_PRIORITY_BOUNDS,   ///< A boundary violation in requested priority.
    ERROR_PRIORITY_QUERY,    ///< A failure when querying priority.
    ERROR_PRIORITY_VALIDATE, ///< A failure to validate priority.
    ERROR_PARAM_QUERY,       ///< A failure querying a scheduling parameter.
    ERROR_PARAM_UPDATE,      ///< A failure updating a scheduling parameter.
    ERROR_GENERAL            ///< A general error in scheduling.
  };

  // The set of API scheduling policies.
  enum policy_e {
    PRIORITY,
    PROGRESS
  };

  // API level scheduling
  static thread_p schedule( std::vector<thread_p>& runqueue );

  static void process_notifications( thread_p& current_thread, std::vector<thread_p>& runqueue, std::vector<thread_p>& waitqueue );

  static void process_wakeups( std::vector<thread_p>& runqueue, std::vector<thread_p>& waitqueue );

  static void step_system( thread_p& current_thread, std::vector<thread_p>& runqueue, std::vector<thread_p>& waitqueue );

  // POSIX level scheduling
  static error_e create( pid_t& pid, const int& priority_offset, const cpu_id_t& cpu, const std::string& program, const std::string& args ); 

  static error_e suspend( const pid_t& pid );

  static error_e resume( const pid_t& pid );

  static error_e terminate( const pid_t& pid );

  static error_e set_realtime_policy( const pid_t& pid, int& priority );

  static error_e set_realtime_policy( const pid_t& pid, int& priority, const int& offset );

  static error_e get_realtime_min_priority( int& priority );

  static error_e get_realtime_max_priority( int& priority );

  static error_e get_priority( const pid_t& pid, int& priority );

  static error_e set_priority( const pid_t& pid, int& priority );

  static error_e decrease_realtime_priority( const pid_t& pid, int& priority, const int& offset, const int& min_priority );

  static error_e increase_realtime_priority( const pid_t& pid, int& priority, const int& offset, const int& max_priority );

  // POSIX pthread scheduling
  static error_e create( pthread_t& thread, const int& priority, worker_f worker );
 
  static error_e get_priority( const pthread_t& thread, int& priority );

  static error_e set_priority( const pthread_t& thread, int& prioirty );

  static error_e decrease_realtime_priority( const pthread_t& thread, int& priority, const int& offset, const int& min_priority );

  static error_e increase_realtime_priority( const pthread_t& thread, int& priority, const int& offset, const int& max_priority );

};

//-----------------------------------------------------------------------------

#endif // _SCHEDULER_H_