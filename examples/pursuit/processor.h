#ifndef _PROCESSOR_H_
#define _PROCESSOR_H_

//-----------------------------------------------------------------------------

#include "timesink.h"

//-----------------------------------------------------------------------------

class processor_c : public timesink_c {
public:
  processor_c( const char* name );
  virtual ~processor_c( void );

  virtual type_e type( void ) { return PROCESSOR; }


}; 

//-----------------------------------------------------------------------------

#endif // _PROCESSOR_H_
