#ifndef _DYNAMICS_H_
#define _DYNAMICS_H_

#include "timesink.h"

class dynamics_c : public timesink_c {
public:

  dynamics_c( void );
  virtual ~dynamics_c( void );

  virtual type_e type( void ) { return DYNAMICS; }

  virtual void dispatch( thread_p& current_thread );
  virtual void terminate( void );

  void step( const realtime_t& dt );
};

#endif // _DYANMICS_H_
