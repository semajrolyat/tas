#include "thread_heap.h"

#include <assert.h>

//-----------------------------------------------------------------------------
thread_heap_c::thread_heap_c( void ) {
  _empty = true;
}

//-----------------------------------------------------------------------------
thread_heap_c::~thread_heap_c( void ) {

}

//-----------------------------------------------------------------------------
bool thread_heap_c::empty( void ) {
  return _empty;
}

//-----------------------------------------------------------------------------
unsigned thread_heap_c::size( void ) {
  return _heap.size();
}

//-----------------------------------------------------------------------------
thread_p thread_heap_c::element( const unsigned& i ) {
  // sanity check that in range
  assert( i < _heap.size() );

  return _heap[i];
}

//-----------------------------------------------------------------------------
thread_heap_c::error_e thread_heap_c::remove( const unsigned& i, thread_p& thread ) {
  // sanity check that in range
  assert( i < _heap.size() );

  // set the return value
  thread = _heap[i];

  // remove the element
  _heap.erase( _heap.begin() + i );
  thread->enqueued = false;

  // reheap
  if( _heap.size() == 0 ) {
    _empty = true;
  } else {
    // maybe superfluous as the heap collapses down but do not want to assume 
    // about the internal organization of the container
    try {
      std::make_heap( _heap.begin(), _heap.end(), compare_thread_p_t() );
    } catch( std::exception ex ) {
      return ERROR_MAKEHEAP;
    }
  }

  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
thread_heap_c::error_e thread_heap_c::push( thread_p thread ) {
  
  // insert the thread into the heap
  _heap.push_back( thread );
  thread->enqueued = true;

  // if the heap was empty before the operation
  if( _empty ) {
    // make sure the system knows its a heap
    try {
      std::make_heap( _heap.begin(), _heap.end(), compare_thread_p_t() );
    } catch( std::exception ex ) {
      return ERROR_MAKEHEAP;
    }
    // and unset the empty flag
    _empty = false;
  } else {
    // otherwise reheap
    try {
      std::push_heap( _heap.begin(), _heap.end(), compare_thread_p_t() );
    } catch( std::exception ex ) {
      return ERROR_PUSHHEAP;
    }
  }

  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
thread_heap_c::error_e thread_heap_c::pop( thread_p& thread ) {

  // check if the heap is empty and return an error if it is
  if( _heap.size() == 0 ) return ERROR_EMPTY;

  try {
    // Note: pop_heap only works correctly if front is accessed immediately
    // before the pop.  If any statements are between then won't pop.
    thread = _heap.front();
    std::pop_heap( _heap.begin(), _heap.end(), compare_thread_p_t() );
  } catch( std::exception ex ) {
    return ERROR_POPHEAP;
  }
  thread->enqueued = false;
  _heap.pop_back();

  // check if the heap is now empty post pop and set the flag
  if( _heap.size() == 0 ) _empty = true;

  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
thread_heap_c::error_e thread_heap_c::top( thread_p& thread ) {

  // check if the heap is empty and return an error if it is
  if( _heap.size() == 0 ) return ERROR_EMPTY;

  // set the top
  thread = _heap.front();

  // success
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
