#include "cpu.h"

#include <string.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include "time.h"

//-----------------------------------------------------------------------------
/// Reads the system information of cpu configurations.
/// @param cpus the set of cpudata_t returned by a read operation.
/// @return indicator of operation success of specified error.
// TODO: right now only parsing processor and speed information.  Expand to
//       include more comprehensive set of processor specs when necessary
cpu_c::error_e cpu_c::read( std::vector<cpudata_t>& cpus ) {
  char buffer[1024];
  cpudata_t cpudata;
  std::stringstream filename;
  char* field, *value;
  char delim[] = ":";
  FILE* fp;

  // clear any possible cpudata in cpus
  cpus.clear();

  // reading from system /proc/cpuinfo
  filename << "/proc/cpuinfo";
  // open the file for reading
  fp = fopen( filename.str().c_str(), "r" );
  // if failed to open return open error
  if( fp == 0 ) return ERROR_OPEN;

  // TODO: improve error checking and return read error if one occurs
  //       but make sure to close fp before bombing out
  while( fgets( buffer, 1024, fp ) != NULL ) {
    //fputs( buffer, stdout );            // for debugging

    // tokenize the field and the value
    field = strtok( buffer, delim );   
    value = strtok( NULL, delim );

    if( value == NULL ) {
      // the delimiter between cpuinfos is a blank line so push cpudata
      // Note: field won't be NULL, but value will be
      //printf( "adding cpu\n" );         // for debugging
      cpus.push_back( cpudata );
    } else {
      // found a field for current cpu so parse
      std::string sfield( field );
      if( sfield.find( "processor" ) != std::string::npos ) {
        //printf( "processor: %s\n", field );   // for debugging
        cpudata.processor = atoi( value );
      }
      if( sfield.find( "cpu MHz" ) != std::string::npos ) {
        //printf( "cpu MHz: %s\n", field );     // for debugging
        cpudata.mhz = atof( value );
      }
    }
  }

  fclose( fp );

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Gets the speed for a specified cpu.
/// @param cpu_hz the frequency of the specified cpu.
/// @param cpu the cpu to get the speed of.
/// @return indicator of operation success of specified error.
cpu_c::error_e cpu_c::get_speed( cpu_speed_t& cpu_hz, const cpu_id_t& cpu ) {
  error_e error;
  unsigned int idx;
  std::vector<cpudata_t> cpus;

  // cache the cpu info from the system
  error = read( cpus );   
  if( error != ERROR_NONE ) return error;

  //printf( "cpuinfo loaded\n" );          // for debugging

  // iterate over the system processors looking for the desired processor
  for( unsigned int i = 0; i < cpus.size( ); i++ ) {
    //printf( "processor: %d\n", cpuinfo.cpus.at( i ).processor );
    if( cpus.at( i ).processor == cpu ) {
      // desired processor found
      idx = i;
      break;
    }
    // if the processor not found on the last iteration, return a no exist error
    if( i == cpus.size( ) - 1 ) return ERROR_NOEXIST;
  }

  // otherwise, the processor has been found
  // Note: inaccuracy in base measure and innaccuracy in fp conversion
  cpu_hz = (unsigned long long) ( cpus.at( idx ).mhz * USECS_PER_SEC );
  // down and dirty fix to fp inaccuracy.
  // TODO : use a much better fix
  if( cpu_hz % 2 == 1 ) cpu_hz++;

  // printf( "cpu speed (Hz): %lld\n", cpu_hz );

  // calibration code that wasn't quite right at the time, revisit.
  //unsigned long long cal_hz = calibrate_cycles_per_second( );
  //printf( "calibrated cpu speed (Hz): %lld\n", cal_hz );

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Restrict the specified process to run only in the prescribed cpu. 
/// @param pid the process to bind.
/// @param cpu the cpu to bind to.
/// @return indicator of operation success of specified error.
cpu_c::error_e cpu_c::bind( const pid_t& pid, const cpu_id_t& cpu ) {
  cpu_set_t cpuset_mask;
  // zero out the cpu set
  CPU_ZERO( &cpuset_mask );
  // bind the processor with the zeroed mask 
  CPU_SET( cpu, &cpuset_mask );

  // set the affinity of the process to the mask containing the processor
  // this effectively binds the process to the processor
  if( sched_setaffinity( pid, sizeof(cpuset_mask), &cpuset_mask ) == -1 ) {
    // there was an error setting the affinity for the coordinator
    // NOTE: can check errno if this occurs
    return ERROR_BIND;
  }

//  // testing sanity check ... TO BE COMMENTED
//  int ret = sched_getaffinity( 0, sizeof(cpuset_mask), &cpuset_mask );
//  printf( " sched_getaffinity = %d, cpuset_mask = %08lx\n", sizeof(cpuset_mask), cpuset_mask );

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------

