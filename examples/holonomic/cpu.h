/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

cpu.h
-----------------------------------------------------------------------------*/

#ifndef _CPU_H_
#define _CPU_H_

//-----------------------------------------------------------------------------

#include <assert.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>


//-----------------------------------------------------------------------------

enum cpuinfo_err_e {
    CPUINFO_ERROR_NONE = 0,
    CPUINFO_ERROR_OPEN,
    CPUINFO_ERROR_READ
};

//-----------------------------------------------------------------------------
// Ubuntu (Linux) 12.04
struct cpu_t {
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

class cpuinfo_c {
public:
    std::vector<cpu_t> cpus;

    //-------------------------------------------------------------------------

    cpuinfo_c( void );

    //------------------------------------------------------------------------

    virtual ~cpuinfo_c( void );

    //-------------------------------------------------------------------------

    cpuinfo_err_e load( void );
};

//-----------------------------------------------------------------------------

cpuinfo_err_e get_cpu_frequency( unsigned long long& cpu_hz, const int& cpu_id);

//-----------------------------------------------------------------------------

#endif // _CPU_H_
