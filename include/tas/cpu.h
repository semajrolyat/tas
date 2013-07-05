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

cpuinfo_err_e get_cpu_frequency( unsigned long long& cpu_hz, const unsigned int& cpu_id ) {
    cpuinfo_c cpuinfo;

    if( cpuinfo.load( ) != CPUINFO_ERROR_NONE ) {
        //sprintf( strbuffer, "(coordinator.cpp) read_cpuinfo() failed calling cpuinfo_c.load()\n" );
        //error_log.write( strbuffer );
        return CPUINFO_ERROR_READ;
    }
    printf( "cpuinfo loaded\n" );

    for( unsigned int i = 0; i < cpuinfo.cpus.size( ); i++ ) {
        //printf( "processor: %d\n", cpuinfo.cpus.at( i ).processor );
        if( cpuinfo.cpus.at( i ).processor == cpu_id ) {
            //cpu_speed_mhz = cpuinfo.cpus.at( i ).mhz;
            // Note: fp conversion will most likely result in inaccuracy
            //cpu_speed_hz = (unsigned long long) (cpu_speed_mhz * 1E6);
            cpu_hz = (unsigned long long) ( cpuinfo.cpus.at( i ).mhz * 1E6 );
            // down and dirty fix to fp inaccuracy.
            // TODO : use a much better fix
            //if( cpu_speed_hz % 2 == 1 ) cpu_speed_hz++;
            if( cpu_hz % 2 == 1 ) cpu_hz++;
            break;
        }
        if( i == cpuinfo.cpus.size( ) - 1 ) {
            //sprintf( strbuffer, "(coordinator.cpp) read_cpuinfo() failed to find default processor in cpus.\n" );
            //error_log.write( strbuffer );
            return CPUINFO_ERROR_READ;
        }
    }
    //printf( "cpu speed (MHz): %f\n", cpu_speed_mhz );
    printf( "cpu speed (Hz): %lld\n", cpu_hz );

    //unsigned long long cal_hz = calibrate_cycles_per_second( );
    //printf( "calibrated cpu speed (Hz): %lld\n", cal_hz );

    return CPUINFO_ERROR_NONE;
}

//-----------------------------------------------------------------------------

#endif // _CPU_H_
