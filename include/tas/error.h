/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

error.h
-----------------------------------------------------------------------------*/

#ifndef _ERROR_H_
#define _ERROR_H_

#include <cstdio>
#include <string>
#include <sstream>

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
std::string error_string_bad_read( const std::string& txt, const int& err ) {
    std::stringstream ss;
    char buffer[256];

    ss << txt << " errno: %s\n";
    sprintf( buffer, ss.str().c_str(), (err == EAGAIN) ? "EAGAIN" :
                                       (err == EBADF) ? "EBADF" :
                                       (err == EFAULT) ? "EFAULT" :
                                       (err == EINTR) ? "EINTR" :
                                       (err == EINVAL) ? "EINVAL" :
                                       (err == EIO) ? "EIO" :
                                       (err == EISDIR) ? "EISDIR" :
                                       "UNDEFINED"
    );
    return std::string( buffer );
}

//-----------------------------------------------------------------------------
/// Common error message for failing to write to file descriptor
std::string error_string_bad_write( const std::string& txt, const int& err ) {
    std::stringstream ss;
    char buffer[256];

    ss << txt << " errno: %s\n";
    sprintf( buffer, ss.str().c_str(), (err == EAGAIN) ? "EAGAIN" :
                                        (err == EBADF) ? "EBADF" :
                     //					(err == EDESTADDRREQ) ? "EDESTADDREQ" :
                     //					(err == EDQUOT) ? "EDQUOT" :
                                        (err == EFAULT) ? "EFAULT" :
                                        (err == EFBIG) ? "EFBIG" :
                                        (err == EINTR) ? "EINTR" :
                                        (err == EINVAL) ? "EINVAL" :
                                        (err == EIO) ? "EIO" :
                                        (err == ENOSPC) ? "ENOSPC" :
                                        (err == EPIPE) ? "EPIPE" :
                                        "UNDEFINED"
    );
    return std::string( buffer );
}

//-----------------------------------------------------------------------------

#endif // _ERROR_H_
