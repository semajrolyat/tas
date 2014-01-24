/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

log.h
-----------------------------------------------------------------------------*/

#ifndef _LOG_H_
#define _LOG_H_

//-----------------------------------------------------------------------------

#include <assert.h>
#include <string>
//#include <iostream>
//#include <fstream>

// Linux using file descriptors
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

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
int __open( const char* path, const int& oflags, const int& mode );
//-----------------------------------------------------------------------------
ssize_t __write( const int& fd, const void* buf, const size_t& count );
//-----------------------------------------------------------------------------
int __close( const int& fd );
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

    int get_fd( void );
    bool get_opened( void );
    log_channel_e get_channel( void );

    //-------------------------------------------------------------------------
    /// Default Constructor
    log_c( void );
    //------------------------------------------------------------------------
    /// Constructor for connecting to physical file
    log_c( const std::string& filename );
    //------------------------------------------------------------------------
    /// Constructor for connecting to STDOUT or STDERR
    log_c( const log_channel_e& channel );
    //-------------------------------------------------------------------------
    /// Constructor for connecting to an already opened file descriptor
    log_c( const int& fd );
    //-------------------------------------------------------------------------
    /// Destructor
    virtual ~log_c( void );

    //-------------------------------------------------------------------------
    /// Opens the channel specified during construction.
    /// Note: it is not necessary to call open if connecting to an already open
    /// file descriptor
    log_err_e open( void );
    //-------------------------------------------------------------------------
    /// Writes the string s to the file descriptor
    log_err_e write( const std::string& s );
    //-------------------------------------------------------------------------
    /// Writes the string s followed by the end line character to the file
    /// descriptor
    log_err_e writeln( const std::string& s );
    //-------------------------------------------------------------------------
    /// Closes an open file descriptor
    /// Note: for physical files the file is closed, but for all other channels
    /// this instance is simply detached.  Therefore for UNKNOWN cases the origin
    /// log_c is the one that actually closes the fd.  If the fd was created
    /// outside of the log_c framework, it would still need to be closed.
    log_err_e close( void );
};

//-----------------------------------------------------------------------------

#endif // _LOG_H_
