#ifndef _TYPES_H_
#define _TYPES_H_

#include <boost/shared_ptr.hpp>

class thread_c;
typedef boost::shared_ptr<thread_c> thread_p;

class message_c;
typedef boost::shared_ptr<message_c> message_p;

typedef void (*select_f)( void );
typedef void (*read_notifications_f)( void );


#endif // _TYPES_H_