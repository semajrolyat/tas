/*-----------------------------------------------------------------------------
time.h encapsulates all time operations including API types and queries and 
POSIX types and queries.

author: James Taylor : jrt@gwu.edu
-----------------------------------------------------------------------------*/

#ifndef _TIME_H_
#define _TIME_H_

//-----------------------------------------------------------------------------

#include <sys/time.h>           // POSIX time

#include "cpu.h"

//-----------------------------------------------------------------------------

// defines to ease conversions from one type to another
#define NSECS_PER_SEC 1E9       // nanoseconds per second
#define USECS_PER_SEC 1E6       // microseconds per second

//-----------------------------------------------------------------------------
/// realtime type in seconds aliasing a system double.
typedef double realtime_t;
/// timestamp type in clock ticks aliasing rdtsc values dependent on cpu speed.
typedef unsigned long long timestamp_t;

//-----------------------------------------------------------------------------

/// Assembly call to get current content of rdtsc register
#define rdtscll(val) __asm__ __volatile__("rdtsc" : "=A" (val))

//-----------------------------------------------------------------------------

timestamp_t generate_timestamp( void );

//-----------------------------------------------------------------------------
// Conversion functions
//-----------------------------------------------------------------------------

double cycles_to_seconds( const unsigned long long& cycles, const cpu_speed_t& cpu_hz ); 

unsigned long long cycles_to_nanoseconds( const unsigned long long& cycles, const cpu_speed_t& cpu_hz );

realtime_t timestamp_to_realtime( const timestamp_t& ts, const cpu_speed_t& cpu_hz );

double timespec_to_real( const struct timespec& ts ); 

realtime_t timespec_to_realtime( const struct timespec& ts );

struct timespec nanoseconds_to_timespec( const unsigned long long& nsec );
 
//-----------------------------------------------------------------------------

#endif // _TIME_H_
