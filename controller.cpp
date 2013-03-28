/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

This is a simple program to act as the controller process for testing purposes.
It only prints the current system time to the console after doing a relatively
intensive computation to simulate time passing and processer load
-----------------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <signal.h>
#include <stdexcept>
#include <sys/time.h>
#include <sys/resource.h>
#include <errno.h>
#include <fstream>

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

#define NSECS_PER_SEC 1E9
#define USECS_PER_SEC 1E6

//#define ITS_TO_WAIT             850000              // roughly 10 its per second on dev machine
#define ITS_TO_WAIT             280000              // roughly 30 its per second on dev machine

int main( int argc, char* argv[] ) {

    struct timeval tv_timenow;
    struct timeval tv_timelast;

    int it = 1;
    int lasttime = 0;
    int thistime = 0;

    while( 1 ) {
        tv_timelast = tv_timenow;

        double x = 1.1;
        for( int i = 0; i < ITS_TO_WAIT; i++ ) {
            x *= x;
        }

        gettimeofday( &tv_timenow, NULL );
        thistime = tv_timenow.tv_sec;

        if( lasttime == 0 ) {
            lasttime = thistime;
        } else if( lasttime != thistime ) {
            it = 1;
            lasttime = thistime;
        } else {
            it++;
        }

        int dusec;
        int dsec = tv_timenow.tv_sec - tv_timelast.tv_sec;

        if( dsec > 0 ) {
            dusec = USECS_PER_SEC - tv_timelast.tv_usec + tv_timenow.tv_usec;
        } else {
            dusec = tv_timenow.tv_usec - tv_timelast.tv_usec;
        }

        printf( "%03d : %d : %d : %d\n", it, tv_timenow.tv_sec, tv_timenow.tv_usec, dusec );
    }

    return 0;
}
