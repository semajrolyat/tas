#ifndef _NOTIFICATION_H_
#define _NOTIFICATION_H_

//-----------------------------------------------------------------------------

#include "time.h"

//-----------------------------------------------------------------------------
struct notification_t {

  enum source_e {
    TIMER,                  ///< Notification originated from a timer.
    WAKEUP,                 ///< Notification originated from block detection.
    CLIENT,                 ///< Notification originated from a client process.
    SERVER                  ///< Notification originated from the coordinator.
  };

  enum type_e {
    OPEN,                   ///< Client is initializing
    CLOSE,                  ///< Client is closing
    IDLE,		    ///< Client requests a block
    READ,		    ///< Client requests state
    WRITE		    ///< Client has written a command
  };

  timestamp_t     ts;       ///< Timestamp when the notification was created
  source_e        source;   ///< Source of the notification
  type_e          type;     ///< Type of notification
  cycle_t         period;   ///< Regulated time between actions
  pid_t           pid;      ///< Process that originated the notification
};

//-----------------------------------------------------------------------------

#endif // _NOTIFICATION_H_
