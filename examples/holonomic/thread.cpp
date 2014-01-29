#include "thread.h"

//-----------------------------------------------------------------------------
thread_c::thread_c( void ) {
  pid = 0;

  type = UNDEFINED;
  status = THREAD_NEW;
  priority = 0;
  exec_time.seconds = 0.0;

  initialized = false;
  PERIOD_NSEC = 0;
}

//-----------------------------------------------------------------------------
thread_c::~thread_c( void ) {

}

//-----------------------------------------------------------------------------
void thread_c::execute( void ) {
  resume();
}

//-----------------------------------------------------------------------------
void thread_c::suspend( void ) {
  if( type == PROCESS ) 
    kill( pid, SIGSTOP );

  status = THREAD_READY;
}

//-----------------------------------------------------------------------------
void thread_c::resume( void ) {
  status = THREAD_RUNNING;

  if( type == PROCESS )
    kill( pid, SIGCONT );

}

//-----------------------------------------------------------------------------
void thread_c::shutdown( void ) {
  status = THREAD_TERMINATED;

  if( type == PROCESS )
    kill( pid, SIGTERM );
    //kill( pid, SIGKILL );
}

//-----------------------------------------------------------------------------

