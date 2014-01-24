/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

error.h
-----------------------------------------------------------------------------*/

#ifndef _ERROR_H_
#define _ERROR_H_

#include <cstdio>
#include <string>
#include <sstream>
#include <errno.h>

//-----------------------------------------------------------------------------

/// Very modest error enumeration.  Classes may have more detailed error
/// facilities, but this general enum provides fundamental flags for generalized
/// use.
enum error_e {
    ERROR_NONE = 0,
    ERROR_FAILED
};


//-----------------------------------------------------------------------------
/// Common error message for failing to read from file descriptor
std::string error_string_bad_read( const std::string& txt, const int& err );
//-----------------------------------------------------------------------------
/// Common error message for failing to write to file descriptor
std::string error_string_bad_write( const std::string& txt, const int& err );
//-----------------------------------------------------------------------------

#endif // _ERROR_H_
