#include "../thread_heap.h"

#include "../timesink.h"
#include "../processor.h"
#include "../osthread.h"

#include <string>
#include <stdio.h>
#include <stdlib.h>

//-----------------------------------------------------------------------------
struct compare_int_t {
  bool operator()( const int& l, const int& r ) const {
    return l > r;
  }
};

//-----------------------------------------------------------------------------
void print_err( std::string function, thread_heap_c::error_e err, int pos ) {
  std::string err_info;
  switch( err ) {
  case thread_heap_c::ERROR_PUSHHEAP:
    err_info = "ERROR_PUSHHEAP";
    break;
  case thread_heap_c::ERROR_POPHEAP:
    err_info = "ERROR_POPHEAP";
    break;
  case thread_heap_c::ERROR_MAKEHEAP:
    err_info = "ERROR_MAKEHEAP";
    break;
  case thread_heap_c::ERROR_EMPTY:
    err_info = "ERROR_EMPTY";
    break;
  default:
    err_info = "ERROR_UNDEFINED";
    break;
  }
  printf( "thread_heap_c error[%s], function[%s] at %d\n", err_info.c_str(), function.c_str(), pos );
}

//-----------------------------------------------------------------------------
void test_processors( void ) {
  thread_heap_c heap;
  thread_heap_c::error_e err;
  boost::shared_ptr<processor_c> cpu1;
  boost::shared_ptr<processor_c> cpu2;
  boost::shared_ptr<processor_c> cpu3;

  cpu1 = boost::shared_ptr<processor_c>( new processor_c() );
  cpu1->name = "cpu1";
  cpu1->progress = 100;
  cpu1->priority = 2;

  cpu2 = boost::shared_ptr<processor_c>( new processor_c() );
  cpu2->name = "cpu2";
  cpu2->progress = 1;
  cpu2->priority = 1;

  cpu3 = boost::shared_ptr<processor_c>( new processor_c() );
  cpu3->name = "cpu3";
  cpu3->progress = 50;
  cpu3->priority = 0;

  err = heap.push( cpu1 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_processors", err, 58 );
  err = heap.push( cpu2 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_processors", err, 60 );
  err = heap.push( cpu3 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_processors", err, 62 );

  boost::shared_ptr<thread_c> t1, t2, t3, t4, t5, t6, t7, t8;
  err = heap.top( t1 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_processors", err, 66 );
  printf( "(test_processors) top of heap=%s\n", t1->name.c_str() );

  err = heap.pop( t2 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_processors", err, 66 );
  err = heap.top( t3 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_processors", err, 66 );
  printf( "(test_processors) popped %s off the heap.  Top now %s and size %u.\n", t2->name.c_str(), t3->name.c_str(), heap.size() );

  unsigned count = heap.size();
  printf( "(test_processors) heap contains: " );
  for( unsigned i = 0; i < count; i++ ) {
    if( i > 0 ) printf( "," );
    t4 = heap.element( i );
    printf( "%s", t4->name.c_str() );
  }
  printf( "\n" );

  err = heap.remove( 1, t5 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_processors", err, 66 );
  err = heap.top( t6 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_processors", err, 66 );
  printf( "(test_processors) removed %s out of the heap.  Top now %s and size %u.\n", t5->name.c_str(), t6->name.c_str(), heap.size() );
 
  err = heap.pop( t7 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_processors", err, 66 );
  printf( "(test_processors) popped %s off the heap.  size %u.\n", t7->name.c_str(), heap.size() );
 
  err = heap.push( cpu1 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_processors", err, 58 );
  err = heap.push( cpu2 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_processors", err, 60 );
  err = heap.push( cpu3 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_processors", err, 62 );
   
  err = heap.top( t8 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_processors", err, 66 );
  printf( "(test_processors) pushed all cpu's back onto heap.  top of heap=%s  size %u.\n", t8->name.c_str(), heap.size() );
 
}

//-----------------------------------------------------------------------------
void test_timesinks( void ) {
  thread_heap_c heap;
  thread_heap_c::error_e err;
  boost::shared_ptr<timesink_c> ts1;
  boost::shared_ptr<timesink_c> ts2;
  boost::shared_ptr<timesink_c> ts3;

  ts1 = boost::shared_ptr<timesink_c>( new timesink_c( scheduler_c:: PRIORITY ) );
  ts1->name = "ts1";
  ts1->progress = 100;
  ts1->priority = 2;

  ts2 = boost::shared_ptr<timesink_c>( new timesink_c( scheduler_c:: PROGRESS ) );
  ts2->name = "ts2";
  ts2->progress = 1;
  ts2->priority = 1;

  ts3 = boost::shared_ptr<timesink_c>( new timesink_c( scheduler_c:: PROGRESS ) );
  ts3->name = "ts3";
  ts3->progress = 50;
  ts3->priority = 0;

  err = heap.push( ts1 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_timesinks", err, 95 );
  err = heap.push( ts2 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_timesinks", err, 97 );
  err = heap.push( ts3 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_timesinks", err, 99 );

  boost::shared_ptr<thread_c> thread;
  err = heap.top( thread );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_timesinks", err, 103 );

  printf( "(test_timesinks) top of heap=%s\n", thread->name.c_str() );
}

//-----------------------------------------------------------------------------
void test_osthreads( void ) {
  thread_heap_c heap;
  thread_heap_c::error_e err;
  boost::shared_ptr<osthread_c> os1;
  boost::shared_ptr<osthread_c> os2;
  boost::shared_ptr<osthread_c> os3;

  os1 = boost::shared_ptr<osthread_c>( new osthread_c() );
  os1->name = "os1";
  os1->progress = 3;
  os1->priority = 2;

  os2 = boost::shared_ptr<osthread_c>( new osthread_c() );
  os2->name = "os2";
  os2->progress = 1;
  os2->priority = 3;

  os3 = boost::shared_ptr<osthread_c>( new osthread_c() );
  os3->name = "os3";
  os3->progress = 2;
  os3->priority = 1;

  err = heap.push( os1 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_osthreads", err, 132 );
  err = heap.push( os2 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_osthreads", err, 134 );
  err = heap.push( os3 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_osthreads", err, 136 );

  boost::shared_ptr<thread_c> t1, t2, t3, t4, t5, t6, t7, t8;
  err = heap.top( t1 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_osthreads", err, 66 );
  printf( "(test_osthreads) top of heap=%s\n", t1->name.c_str() );

  err = heap.pop( t2 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_osthreads", err, 66 );
  err = heap.top( t3 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_osthreads", err, 66 );
  printf( "(test_osthreads) popped %s off the heap.  Top now %s and size %u.\n", t2->name.c_str(), t3->name.c_str(), heap.size() );

  unsigned count = heap.size();
  printf( "(test_osthreads) heap contains: " );
  for( unsigned i = 0; i < count; i++ ) {
    if( i > 0 ) printf( "," );
    t4 = heap.element( i );
    printf( "%s", t4->name.c_str() );
  }
  printf( "\n" );

  err = heap.remove( 1, t5 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_osthreads", err, 66 );
  err = heap.top( t6 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_osthreads", err, 66 );
  printf( "(test_osthreads) removed %s out of the heap.  Top now %s and size %u.\n", t5->name.c_str(), t6->name.c_str(), heap.size() );
 
  err = heap.pop( t7 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_osthreads", err, 66 );
  printf( "(test_osthreads) popped %s off the heap.  size %u.\n", t7->name.c_str(), heap.size() );
 
  err = heap.push( os1 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_osthreads", err, 58 );
  err = heap.push( os2 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_osthreads", err, 60 );
  err = heap.push( os3 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_osthreads", err, 62 );
   
  err = heap.top( t8 );
  if( err != thread_heap_c::ERROR_NONE ) print_err( "test_osthreads", err, 66 );
  printf( "(test_osthreads) pushed all threads's back onto heap.  top of heap=%s  size %u.\n", t8->name.c_str(), heap.size() );
}

//-----------------------------------------------------------------------------
int main( void ) {

  test_processors();  
  test_timesinks();  
  test_osthreads();  
/*
  int myints[] = {10,20,30,5,15};
  std::vector<int> v(myints,myints+5);

  std::make_heap (v.begin(),v.end(), compare_int_t() );
  std::cout << "initial max heap   : " << v.front() << '\n';

  std::pop_heap (v.begin(),v.end()); v.pop_back();
  std::cout << "max heap after pop : " << v.front() << '\n';

  v.push_back(99); std::push_heap (v.begin(),v.end());
  std::cout << "max heap after push: " << v.front() << '\n';

  std::sort_heap (v.begin(),v.end());

  std::cout << "final sorted range :";
  for (unsigned i=0; i<v.size(); i++)
    std::cout << ' ' << v[i];

  std::cout << '\n';
*/
  return 0;
}
