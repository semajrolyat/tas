
//-----------------------------------------------------------------------------
// MAPPED IPC CHANNELS
//-----------------------------------------------------------------------------

#define FD_ERROR_LOG				    100

#define FD_COORDINATOR_TO_CONTROLLER_READ_CHANNEL   1000
#define FD_COORDINATOR_TO_CONTROLLER_WRITE_CHANNEL  1001

#define FD_CONTROLLER_TO_COORDINATOR_READ_CHANNEL   1002
#define FD_CONTROLLER_TO_COORDINATOR_WRITE_CHANNEL  1003

#define FD_TIMER_TO_COORDINATOR_READ_CHANNEL   1004
#define FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL  1005

#define FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL   1006
#define FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL  1007


//-----------------------------------------------------------------------------

class channel_c {
public:
    channel_c( void ) { }
    virtual ~channel_c( void ) { }


};

//-----------------------------------------------------------------------------

class notification_c {
public:
    notification_c( void ) { }
    virtual ~notification_c( void ) { }

};

//-----------------------------------------------------------------------------

class event_c {  
public:
    event_c( void ) { }
    virtual ~event_c( void ) { }

};

//-----------------------------------------------------------------------------

