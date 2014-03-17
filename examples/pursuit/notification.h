#ifndef _NOTIFICATION_H_
#define _NOTIFICATION_H_

//-----------------------------------------------------------------------------

#include "time.h"

//-----------------------------------------------------------------------------
struct notification_t {

  enum source_e {
    TIMER, 
    WAKEUP,
    CLIENT 
  };

  enum type_e {
    IDLE,		// explicit block request
    OPEN,               // initializing
    CLOSE,              // exiting
    READ,		// reading state information (state request)
    WRITE		// writing result
  };

  timestamp_t ts;
  realtime_t t;
  source_e source;
  type_e type;
  realtime_t blocktill;
};

//-----------------------------------------------------------------------------

#endif // _NOTIFICATION_H_
