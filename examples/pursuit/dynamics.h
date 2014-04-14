#ifndef _DYNAMICS_H_
#define _DYNAMICS_H_

#include "timesink.h"

#define STEP_SIZE_SECONDS 0.001

class dynamics_c : public timesink_c {
private:
  cycle_t _step_size_cycles;

public:

  dynamics_c( const char* name, const timesink_p& owner, const cpu_speed_t& cpu_speed );
  virtual ~dynamics_c( void );

  virtual type_e type( void ) { return DYNAMICS; }

  virtual void dispatch( thread_p& current_thread );
  virtual void terminate( void );

  void step( const realtime_t& dt );
  void step( const cycle_t& dt );
};

#endif // _DYANMICS_H_
