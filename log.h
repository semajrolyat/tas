/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

log.h
-----------------------------------------------------------------------------*/

#ifndef _LOG_H_
#define _LOG_H_

//-----------------------------------------------------------------------------

#include <assert.h>
#include <string.h>
//#include <iostream>
//#include <fstream>

// Linux using file descriptors
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

//-----------------------------------------------------------------------------

enum log_err_e {
    LOG_ERROR_NONE = 0,
    LOG_ERROR_OPEN,
    LOG_ERROR_CLOSE,
    LOG_ERROR_WRITE
};

//-----------------------------------------------------------------------------

enum log_channel_e {
    LOG_CHANNEL_FILE = 0,
    LOG_CHANNEL_STDOUT,
    LOG_CHANNEL_STDERR
};

//-----------------------------------------------------------------------------

int __open( const char* path, int oflags, int mode ) {
    return open( path, oflags, mode );
}

//-----------------------------------------------------------------------------

ssize_t __write( int fd, const void* buf, size_t count ) {
    return write( fd, buf, count );
}

int __close( int fd ) {
    return close( fd );
}

//-----------------------------------------------------------------------------

class log_c {
private:
    //std::ostream* str;
    int fd;
    bool opened;
    log_channel_e channel;
    //-------------------------------------------------------------------------
public:
    //-------------------------------------------------------------------------

    log_c( void ) { 
	opened = false;
	channel = LOG_CHANNEL_FILE;
    }

    //------------------------------------------------------------------------

    log_c( log_channel_e channel ) {
	opened = false;
	this->channel = channel;
    }

    //-------------------------------------------------------------------------
    
    log_c( log_channel_e channel, const int& fd ) {
	//opened = false;
	this->channel = channel;
	this->fd = fd;

	//could add some assertions
    }

    //-------------------------------------------------------------------------

    virtual ~log_c( void ) { }

    //-------------------------------------------------------------------------
    log_err_e open( void ) {
	assert( channel == LOG_CHANNEL_STDOUT || channel == LOG_CHANNEL_STDERR );

	if( channel == LOG_CHANNEL_STDOUT ) {
	    fd = 1;
	} else if (channel == LOG_CHANNEL_STDERR) {
	    fd = 2;
	}

	opened = true;
    }
    //-------------------------------------------------------------------------

    log_err_e open( std::string filename ) {
	assert( channel == LOG_CHANNEL_FILE );

	if( (fd = __open( filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC,
	                  S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH )) == -1 ) {
	    return LOG_ERROR_OPEN;
	}
	opened = true;
	
        return LOG_ERROR_NONE;
    }

    //-------------------------------------------------------------------------

    log_err_e write( std::string s ) {
	assert( opened );

	if( __write( fd, s.c_str( ), s.size( ) ) == -1 )
	    return LOG_ERROR_WRITE;
	return LOG_ERROR_NONE;
    }

    //-------------------------------------------------------------------------

    log_err_e writeln( std::string s ) {
	// Note: implicit assert( opened ) provided by write

	if( write( s ) == LOG_ERROR_WRITE )
	    return LOG_ERROR_WRITE;

        if( __write( fd, "\n", 1 ) == -1 )
	    return LOG_ERROR_WRITE;

	return LOG_ERROR_NONE;
    }

    //-------------------------------------------------------------------------

     log_err_e close( void ) {
	assert( opened );

	opened = false;

	if( __close( fd ) == -1 ) 
	    return LOG_ERROR_CLOSE;
	return LOG_ERROR_NONE;
    }
};

//-----------------------------------------------------------------------------

#endif // _LOG_H_
