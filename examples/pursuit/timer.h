/*-----------------------------------------------------------------------------
timer.h encapsulates all POSIX realtime timer functionality 

author: James Taylor : jrt@gwu.edu
-----------------------------------------------------------------------------*/

#ifndef _TIMER_H_
#define _TIMER_H_

//-----------------------------------------------------------------------------

#include "time.h"
#include "cpu.h"

#include <signal.h>

//-----------------------------------------------------------------------------

// The signal identifier for the controller's real-time timer
#define RTTIMER_SIGNAL                          SIGRTMIN + 4

//-----------------------------------------------------------------------------

/// Signalhandler function pointer type for use with timers.
typedef void (*sighandler_f)( int signum, siginfo_t *si, void *data );

//-----------------------------------------------------------------------------

class timer_c {
private:
  sigset_t            _rttimer_mask;        // POSIX signal mask
  struct sigaction    _rttimer_sigaction;   // POSIX sigaction
  timer_t             _rttimer_id;          // timer identifier
  bool                _first_arming;        // has this timer armed before

public:
  long long agg_error;                ///< Aggregate error of the timer
  unsigned long long last_overrun;    ///< The amount of the last overrun
  timestamp_t ts_prev_arm;            ///< The time of last arming

  /// The set of timer operation errors.
  enum error_e {
    ERROR_NONE = 0,          ///< Operation completed successfully.
    ERROR_SIGACTION,         ///< A failure to set the signal action.
    ERROR_SIGPROCMASK,       ///< A failure to set the signal mask.
    ERROR_CREATE,            ///< A failure to create the timer.
    ERROR_SETTIME,           ///< A failure to set the time.
    ERROR_GETCLOCKID         ///< A failure to get the clock id.
  };

  timer_c( void );
  virtual ~timer_c( void ); 

  error_e block( void );
  error_e unblock( void );

  error_e create( sighandler_f sighandler, int signum );
  error_e arm( const timestamp_t& ts_req, timestamp_t& ts_arm, const unsigned long long& period_nsec, const cpu_speed_t& cpu_hz );

};

//-----------------------------------------------------------------------------


#endif // _TIMER_H_
