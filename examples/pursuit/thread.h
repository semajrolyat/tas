#ifndef _THREAD_H_
#define _THREAD_H_

//-----------------------------------------------------------------------------

#include <queue>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include "types.h"

#include "time.h"
#include "notification.h"
#include "log.h"

//-----------------------------------------------------------------------------
class thread_c : public boost::enable_shared_from_this<thread_c> {
protected:
  log_c* _info;

public:
  cycle_t temporal_progress;        ///< progress, desired/scheduled time
  cycle_t computational_progress;   ///< progress, actual resource usage time

  bool invalidated;

  int priority;
  cpu_speed_t _cpu_speed;

  timestamp_t     ts_dispatched;
  cycle_t         cy_dselect;

  std::queue<notification_t> message_queue;

  char name[128];

  bool enqueued;

  enum type_e {
    UNDEFINED = 0,
    TIMESINK,
    PROCESSOR,
    OSTHREAD,
    DYNAMICS
  };

  thread_c( void ) { }
  virtual ~thread_c( void ) { }

  virtual type_e type( void ) { return UNDEFINED; }

  virtual void dispatch( thread_p& current_thread ) = 0;
  virtual void terminate( void ) = 0;

};

//-----------------------------------------------------------------------------

#endif // _THREAD_H_
