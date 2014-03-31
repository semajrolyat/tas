#ifndef _THREAD_H_
#define _THREAD_H_

//-----------------------------------------------------------------------------

#include <queue>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include "time.h"
#include "notification.h"
#include "log.h"

#include <string>

//-----------------------------------------------------------------------------
class thread_c;
typedef boost::shared_ptr<thread_c> thread_p;

//-----------------------------------------------------------------------------
class thread_c : public boost::enable_shared_from_this<thread_c> {
protected:
  log_c* _log;

public:
  timestamp_t progress;
  int priority;
  timestamp_t timestamp;
  cpu_speed_t cpu_speed;

  std::queue<notification_t> message_queue;

  std::string name;

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
