#ifndef _OSTHREAD_H_
#define _OSTHREAD_H_

#include "timesink.h"

#include <string>
#include <signal.h>
#include <pthread.h>

#include "types.h"

class osthread_c : public timesink_c {
public:
  int _max_os_priority;
  int _min_os_priority;
  int _os_priority_step;
  int _os_priority;

  pid_t pid;
  std::string program;

  realtime_t run_time;        ///< Time actually running
  realtime_t wait_time;       ///< Time ready to run but not running
  realtime_t block_time;      ///< Time blocking

  // callback functions
  select_f select; 
  read_notifications_f read_notifications;

  osthread_c( void );
  osthread_c( select_f select, read_notifications_f read_notifications, log_c* log );
  virtual ~osthread_c( void );

  virtual type_e type( void ) { return OSTHREAD; }

  virtual void dispatch( thread_p& current_thread );
//  virtual void terminate( void );

  virtual void raise_priority( void );
  virtual void lower_priority( void );
/*
  virtual void block( void );
  virtual void unblock( void );
*/
};

#endif // _OSTHREAD_H_
