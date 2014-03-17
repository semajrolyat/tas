#include "time.h"

//-----------------------------------------------------------------------------
/// Wraps the rdtsc assembly call and macro into a C/C++ function.
/// @return the current timestamp as indicated by the rdtsc register.
timestamp_t generate_timestamp( void ) {
  timestamp_t ts;
  rdtscll( ts );
  return ts;
}

//----------------------------------------------------i-------------------------
/// Conversion from cycles to seconds.  Relies on a floating point computation
/// so there will be inaccuracy in the result, but the error should be a 
/// truncation error so in terms of seconds there will be some small loss that 
/// will accrue over large periods of time.
/// @param cycles a value in terms of cpu cycles.
/// @param cpu_hz the speed of the processor.
/// @return seconds as computed from cycles and the processor speed.
double cycles_to_seconds( const unsigned long long& cycles, const cpu_speed_t& cpu_hz ) {
  return (double)cycles / (double)cpu_hz;
}

//-----------------------------------------------------------------------------
/// Conversion from cycles to nanoseconds.
/// @param cycles a value in terms of cpu cycles.
/// @param cpu_hz the speed of the processor.
/// @return nanoseconds as computed from cycles and the processor speed.
unsigned long long cycles_to_nanoseconds( const unsigned long long& cycles, const cpu_speed_t& cpu_hz ) {
    return (cycles * (unsigned long long) NSECS_PER_SEC) / cpu_hz;
}

//-----------------------------------------------------------------------------
/// Conversion from timestamp_t type to realtime_t type.  Uses cycles_to_seconds
/// so refer to that documentation for any notes on accuracy.
/// @param ts a timestamp.
/// @param cpu_hz the speed of the processor.
/// @return a realtime value converted from the timestamp value.
realtime_t timestamp_to_realtime( const timestamp_t& ts, const cpu_speed_t& cpu_hz ) {
  realtime_t rt;
  rt = cycles_to_seconds( (unsigned long long)ts, cpu_hz );
  return rt;
}

//-----------------------------------------------------------------------------
/// Conversion from POSIX timespec to floating point seconds.  Relies on a 
/// floating point computation so there will be inaccuracy in the result, but 
/// the error should be a truncation error so in terms of seconds there will 
/// be some small loss that will accrue over large periods of time.
/// @param ts a POSIX timespec structure.
/// @return a real value converted from the timespec value.
double timespec_to_real( const struct timespec& ts ) {
  return (double) ts.tv_sec + (double) ts.tv_nsec / (double) NSECS_PER_SEC;
}

//-----------------------------------------------------------------------------
/// Conversion from POSIX timespec to realtime type.  Uses timespec_to_real
/// so refer to that documentation for any notes on accuracy.
/// @param ts a POSIX timespec structure.
/// @return a realtime type converted from the timespec value.
realtime_t timespec_to_realtime( const struct timespec& ts ) {
  return timespec_to_real( ts );
}

//-----------------------------------------------------------------------------

