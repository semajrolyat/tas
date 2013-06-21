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
    LOG_CHANNEL_UNKNOWN,
    LOG_CHANNEL_STDOUT,
    LOG_CHANNEL_STDERR,
    LOG_CHANNEL_FILE
};

//-----------------------------------------------------------------------------

int __open( const char* path, const int& oflags, const int& mode ) {
    return open( path, oflags, mode );
}

//-----------------------------------------------------------------------------

ssize_t __write( const int& fd, const void* buf, const size_t& count ) {
    return write( fd, buf, count );
}

int __close( const int& fd ) {
    return close( fd );
}

//-----------------------------------------------------------------------------

class log_c {
private:
    int fd;
    bool opened;
    log_channel_e channel;
    std::string filename;
    //-------------------------------------------------------------------------
public:
    //-------------------------------------------------------------------------

    int get_fd( void ) { return fd; }
    bool get_opened( void ) { return opened; }
    log_channel_e get_channel( void ) { return channel; }

    //-------------------------------------------------------------------------
    /// Default Constructor
    log_c( void ) { 
	opened = false;
	channel = LOG_CHANNEL_STDOUT;
	fd = -1;
    }

    //------------------------------------------------------------------------
    /// Constructor for connecting to physical file
    log_c( const std::string& filename ) { 
	opened = false;
	channel = LOG_CHANNEL_FILE;
	fd = -1;
	this->filename = filename;
    }

    //------------------------------------------------------------------------
    /// Constructor for connecting to STDOUT or STDERR
    log_c( const log_channel_e& channel ) {
	assert( channel != LOG_CHANNEL_FILE && channel != LOG_CHANNEL_UNKNOWN );

	opened = false;
	this->channel = channel;
	fd = -1;
    }

    //-------------------------------------------------------------------------
    /// Constructor for connecting to an already opened file descriptor
    log_c( const int& fd ) {
	assert( fcntl( fd, F_GETFL) != -1 && errno != EBADF );

	opened = true;
	this->channel = LOG_CHANNEL_UNKNOWN;
	this->fd = fd;
    }

    //-------------------------------------------------------------------------
    /// Destructor
    virtual ~log_c( void ) { }

    //-------------------------------------------------------------------------
    /// Opens the channel specified during construction.
    /// Note: it is not necessary to call open if connecting to an already open
    /// file descriptor
    log_err_e open( void ) {
	//assert( channel != LOG_CHANNEL_UNKNOWN );

	if( channel == LOG_CHANNEL_STDOUT ) {
	    // Note: these channels may have been explicitly closed!
	    fd = 1;
	} else if (channel == LOG_CHANNEL_STDERR) {
	    fd = 2;
	} else if (channel == LOG_CHANNEL_FILE ){
            if( (fd = __open( filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC,
	                      S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH )) == -1 ) {
       		return LOG_ERROR_OPEN;
	    }
	} else {	// LOG_CHANNEL_UNKNOWN
	    // Technically this is not opening the channel as it must already
	    // be opened, but it does validate that the channel is open
	    // (but not currently validating rights).  It is not necessary
	    // to call open on an already opened fd but better to allow and
	    // validate than to have a radically different procedure for 
	    // connecting to the UNKNOWN channel than to the others
	    if( fcntl( fd, F_GETFL) == -1 && errno == EBADF )
	    	return LOG_ERROR_OPEN;
	}

	opened = true;

	return LOG_ERROR_NONE;
    }

    //-------------------------------------------------------------------------
    /// Writes the string s to the file descriptor
    log_err_e write( const std::string& s ) {
	assert( opened );

	if( __write( fd, s.c_str( ), s.size( ) ) == -1 )
	    return LOG_ERROR_WRITE;
	return LOG_ERROR_NONE;
    }

    //-------------------------------------------------------------------------
    /// Writes the string s followed by the end line character to the file
    /// descriptor
    log_err_e writeln( const std::string& s ) {
	// Note: implicit assert( opened ) via write

	if( write( s ) == LOG_ERROR_WRITE )
	    return LOG_ERROR_WRITE;

        if( __write( fd, "\n", 1 ) == -1 )
	    return LOG_ERROR_WRITE;

	return LOG_ERROR_NONE;
    }

    //-------------------------------------------------------------------------
    /// Closes an open file descriptor
    /// Note: for physical files the file is closed, but for all other channels
    /// this instance is simply detached.  Therefore for UNKNOWN cases the origin
    /// log_c is the one that actually closes the fd.  If the fd was created
    /// outside of the log_c framework, it would still need to be closed.
    log_err_e close( void ) {
	assert( opened );

	if( channel == LOG_CHANNEL_FILE )
	    if( __close( fd ) == -1 ) 
	    	return LOG_ERROR_CLOSE;

	opened = false;
	fd = -1;

	return LOG_ERROR_NONE;
    }
};

//-----------------------------------------------------------------------------

#endif // _LOG_H_
