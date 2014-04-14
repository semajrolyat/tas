#ifndef _THREAD_HEAP_H_
#define _THREAD_HEAP_H_

#include "types.h"
#include "thread.h"
#include <vector>
#include <algorithm>
#include <exception>

class thread_heap_c {
private:
  std::vector<thread_p> _heap;
  bool _empty;

public: 
  enum error_e {
    ERROR_NONE,
    ERROR_MAKEHEAP,
    ERROR_PUSHHEAP,
    ERROR_POPHEAP,
    ERROR_EMPTY
  };

  thread_heap_c( void );
  virtual ~thread_heap_c( void );

  error_e push( thread_p thread );
  error_e pop( thread_p& thread );
  error_e top( thread_p& thread );

  unsigned size( void );
  thread_p element( const unsigned& i );
  error_e remove( const unsigned& i, thread_p& thread );
  bool empty( void );
};

//-----------------------------------------------------------------------------
struct compare_thread_p_t {
  bool operator()( const thread_p& t1, const thread_p& t2 ) const {
    if( t1->type() == thread_c::PROCESSOR && t2->type() == thread_c::PROCESSOR ) {
      // Min heap.  Want lowest progress thread at top of heap
      return t1->temporal_progress > t2->temporal_progress;
    } else if( t1->type()==thread_c::OSTHREAD && t2->type()==thread_c::OSTHREAD ) {
      // Max heap.  Want highest priority thread at top of heap
      return t1->priority < t2->priority;
    } else  {
      // Generic catch all for all other cases
      // May need additional specific cases
      return t1->temporal_progress > t2->temporal_progress; 
    }
  }
};

//-----------------------------------------------------------------------------

#endif // _THREAD_HEAP_H_
