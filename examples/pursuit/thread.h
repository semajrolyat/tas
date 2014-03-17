#ifndef _THREAD_H_
#define _THREAD_H_

//-----------------------------------------------------------------------------

#include <queue>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include "time.h"
#include "scheduler.h"
#include "notification.h"

#include <string.h>

//-----------------------------------------------------------------------------
class thread_c;
typedef boost::shared_ptr<thread_c> thread_p;

//-----------------------------------------------------------------------------
class thread_c : public boost::enable_shared_from_this<thread_c> {
public:
  realtime_t progress;
  timestamp_t timestamp;

  int priority;
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
struct compare_thread_p_t {
  bool operator()( const thread_p& t1, const thread_p& t2 ) {
    if( t1->type() == thread_c::PROCESSOR && t2->type() == thread_c::PROCESSOR )
      // Min heap.  Want lowest progress thread at top of heap
      return t1->progress < t2->progress;
    else if( t1->type()==thread_c::OSTHREAD && t2->type()==thread_c::OSTHREAD )
      // Max heap.  Want highest priority thread at top of heap
      return t1->priority > t2->priority;
    else 
      // Generic catch all for all other cases
      // May need additional specific cases
      return t1->progress < t2->progress; 
  }
};

//-----------------------------------------------------------------------------

#endif // _THREAD_H_
