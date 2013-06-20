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

    cpuinfo_c( void ) { }

    //------------------------------------------------------------------------

    virtual ~cpuinfo_c( void ) { }

    //-------------------------------------------------------------------------

    cpuinfo_err_e load( void ) {
        char buffer[1024];

	cpu_t cpu;

	std::stringstream filename;
	filename << "/proc/cpuinfo";
	FILE* fp = fopen( filename.str().c_str(), "r" );
        if( fp == 0 ) return CPUINFO_ERROR_OPEN;

        char* field, *value;
	char delim[] = ":";

	while( fgets( buffer, 1024, fp ) != NULL ) {
	    //fputs( buffer, stdout );

	    field = strtok( buffer, delim );
            value = strtok( NULL, delim );

	    if( value == NULL ) {
		// delimiter between cpuinfos is a blank line
		//printf( "adding cpu\n" );
		cpus.push_back( cpu );
            } else {
		std::string sfield( field );
		if( sfield.find( "processor" ) != std::string::npos ) {
		    //printf( "processor: %s\n", field );
		    cpu.processor = atoi( value );
 		}
		if( sfield.find( "cpu MHz" ) != std::string::npos ) {
		    //printf( "cpu MHz: %s\n", field );
		    cpu.mhz = atof( value );
 		}
     	    }

        }

	fclose( fp );

	return CPUINFO_ERROR_NONE;
    }

};

//-----------------------------------------------------------------------------

#endif // _CPU_H_
