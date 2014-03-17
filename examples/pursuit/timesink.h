#ifndef _TIMESINK_H_
#define _TIMESINK_H_

#include "thread.h"
#include "scheduler.h"

//#include <queue>
//#include <boost/shared_ptr.hpp>


class timesink_c : public thread_c {
public:

  timesink_c( void );
  timesink_c( const scheduler_c::policy_e& policy );
  virtual ~timesink_c( void );

  virtual type_e type( void ) { return TIMESINK; }
  
  virtual void dispatch( thread_p& current_thread );
  virtual void terminate( void );

  thread_p current_thread;
  scheduler_c::policy_e policy;
  std::vector<thread_p> runqueue;    // heap
  std::vector<thread_p> waitqueue;   // heap

}; 

#endif // _TIMESINK_H_
