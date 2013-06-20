/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

log.h
-----------------------------------------------------------------------------*/

#ifndef _LOG_H_
#define _LOG_H_

//-----------------------------------------------------------------------------

#include <assert.h>
#include <string.h>
#include <iostream>
#include <fstream>

//-----------------------------------------------------------------------------

enum log_err_e {
    LOG_ERROR_NONE = 0,
    LOG_ERROR_OPEN,
    LOG_ERROR_WRITE
};

//-----------------------------------------------------------------------------

enum log_channel_e {
    LOG_CHANNEL_FILE = 0,
    LOG_CHANNEL_STDOUT,
    LOG_CHANNEL_STDERR
};

//-----------------------------------------------------------------------------

class log_c {
private:
    std::ostream* str;
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

    virtual ~log_c( void ) { }

    //-------------------------------------------------------------------------
    log_err_e open( void ) {
	assert( channel == LOG_CHANNEL_STDOUT || channel == LOG_CHANNEL_STDERR );

	if( channel == LOG_CHANNEL_STDOUT ) {
	    str = (std::ostream*)&std::cout;
	} else if (channel == LOG_CHANNEL_STDERR) {
	    str = (std::ostream*)&std::cerr;
	}

	opened = true;
    }
    //-------------------------------------------------------------------------

    log_err_e open( std::string filename ) {
	assert( channel == LOG_CHANNEL_FILE );

	std::ofstream* file = new std::ofstream( );

        file->exceptions( std::ofstream::failbit );
	try {
            file->open( filename.c_str( ), std::ofstream::out | std::ofstream::trunc );
        } catch ( std::ofstream::failure ex ) {
            return LOG_ERROR_OPEN;
        }
	opened = true;
	str = file;
        return LOG_ERROR_NONE;
    }

    //-------------------------------------------------------------------------

    void write( std::string s ) {
	assert( opened );

	(*str) << s;
    }

    //-------------------------------------------------------------------------

    void writeln( std::string s ) {
	assert( opened );

	(*str) << s << std::endl;
    }

    //-------------------------------------------------------------------------

    void close( void ) {
	opened = false;

	if( channel == LOG_CHANNEL_FILE ) {
	    std::ofstream* file = (std::ofstream*)str;
            file->close( );
	    delete file;
	}
	str = NULL;
    }
};

//-----------------------------------------------------------------------------

#endif // _LOG_H_
