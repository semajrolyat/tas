/*-----------------------------------------------------------------------------
cpu.h encapsulates all POSIX level cpu queries.

author: James Taylor : jrt@gwu.edu
-----------------------------------------------------------------------------*/

#ifndef _CPU_H_
#define _CPU_H_

//-----------------------------------------------------------------------------

#include <vector>
#include <sys/types.h>

//-----------------------------------------------------------------------------

#define DEFAULT_CPU    0

//-----------------------------------------------------------------------------

/// cpu speed type aliasing cycle type.
typedef unsigned long long cpu_speed_t;
/// cpu identified type aliasing system type.
typedef int cpu_id_t;

//-----------------------------------------------------------------------------
/// Structure to encapsulate the data contained in /proc/cpuinfo.
/// valid for Ubuntu (Linux) 12.04.
struct cpudata_t {
    int processor;
    char* vendor_id;
    char cpu_family;
    int model;
    char* model_name;
    int stepping;
    int microcode;
    double mhz;
    int cache_size;
    int physical_id;
    int siblings;
    int core_id;
    int cores;
    int apicid;
    int initial_apicid;
    bool fdiv_bug;
    bool hlt_bug;
    bool f00f_bug;
    bool coma_bug;
    bool fpu;
    bool fpu_exception;
    int cpuid_level;
    bool wp;
    char* flags;
    double bogomips;
    int clflush_size;
    int cache_alignment;
    char* address_sizes;
    bool power_management;
};

//-----------------------------------------------------------------------------

class cpu_c {
public:

  /// The set of cpu operation errors.
  enum error_e {
    ERROR_NONE = 0,          ///< Operation completed successfully.
    ERROR_OPEN,              ///< A failure to open cpu info through the os.
    ERROR_READ,              ///< A failure to read cpu info from the os.
    ERROR_BIND,              ///< A failure to bind the process to the cpu.
    ERROR_NOEXIST            ///< A failure to find the requested cpu.
  };

  static error_e read( std::vector<cpudata_t>& cpus ); 

  static error_e get_speed( cpu_speed_t& cpu_hz, const cpu_id_t& cpu );

  static error_e bind( const pid_t& pid, const cpu_id_t& cpu );
}; 

//-----------------------------------------------------------------------------

#endif // _CPU_H_
