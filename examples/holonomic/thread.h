#ifndef _THREAD_H_
#define _THREAD_H_

//-------------------------------------------------------------------------

#include <string>
#include <pthread.h>            //POSIX

#include "time.h"
#include "error.h"

//-------------------------------------------------------------------------

enum thread_type_e {
    UNDEFINED,
    THREAD,
    PROCESS
};

//-----------------------------------------------------------------------------

enum thread_status_e {
    THREAD_NEW,                 // created (may need internal initialization though)
    THREAD_READY,               // ready to run (and likely preempted)
    THREAD_RUNNING,             // executing
    THREAD_WAITING,             // blocking on I/O
    THREAD_TERMINATED           // going or gone away
};

//-----------------------------------------------------------------------------

typedef void* (thread_callback_fn)( void* );

// encompasses both the concepts of thread and process and abstracts into a single class
// really more a pure data type than a class at the moment
class thread_c {
public:

  //---------------------------------------------------------------------------

  // a union of
  pid_t pid;                  // process identifier
  std::string program;        // name of external program to run if process
  // and
  pthread_t thread;

  //---------------------------------------------------------------------------

  // universal members
  thread_type_e type;         // type of thread
  thread_status_e status;     // status of thread
  int priority;               // priority of the thread for scheduling

  realtime_t exec_time;

  // management
  bool initialized;
  unsigned long long PERIOD_NSEC;

public:
  thread_c( void ); 
  virtual ~thread_c( void );

  //virtual void execute( void ) = 0;
  virtual void execute( void ) {}
};

//-----------------------------------------------------------------------------

#endif // _THREAD_H_

