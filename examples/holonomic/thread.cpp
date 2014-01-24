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

