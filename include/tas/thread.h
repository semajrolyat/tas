#ifndef THREAD_H
#define THREAD_H

#include <string>

#include <pthread.h>            //POSIX

#include <tas/error.h>
#include <tas/schedule.h>

//-----------------------------------------------------------------------------

enum thread_type_e {
    THREAD,
    PROCESS
};

//-----------------------------------------------------------------------------

enum thread_status_e {
    THREAD_BLOCKED,    // blocked
    THREAD_EXEC,       // running
    THREAD_WAIT        // waiting on IO
};

//-----------------------------------------------------------------------------

typedef void* (thread_callback_fn)( void* );

// encompasses both the concepts of thread and process and abstracts into a single class
// TODO : determine visibility for member variables and provide any interfaces
// for those that get privatized but are still accessible outside, e.g. pid
class thread_c {
private:
    pid_t pid;                  // process identifier
    pthread_t thread;

public:
    thread_type_e type;         // type of thread
    int priority;               // priority of the thread for scheduling
    std::string program;        // name of external program to run if process

    bool suspended;             // is the thread suspended by the scheduler
    bool blocked;               // is the thread blocked waiting on I/O

    //-------------------------------------------------------------------------
public:
    thread_c( void ) {
        suspended = false;
        blocked = false;
    }

    //-------------------------------------------------------------------------

    thread_c( const std::string& program ) {
        suspended = false;
        blocked = false;
        this->program = program;
        type = PROCESS;
    }

    //-------------------------------------------------------------------------

    virtual ~thread_c( void ) {

    }

    //-------------------------------------------------------------------------

    /// Creates the controller process with a priority level one step below the coordinator.
    /// Note: an ERROR_NONE in the coordinator does not really guarantee that the controller
    /// started properly
    error_e create( const int& processor ) {

        if( type == PROCESS ) {
            pid = fork( );

            // handle any fork error
            if( pid < 0 ) {
                /*
                sprintf( errstr, "(coordinator.cpp) fork_controller() failed calling fork()\n" );
                error_log.write( errstr );
                */
                return ERROR_FAILED;
            }

            // if this is still the parent process then get out.
            if( pid > 0 ) return ERROR_NONE;

            // start of the child process
            pid = getpid( );

            if( set_cpu( pid, processor ) != ERROR_NONE ) {
                /*
                sprintf( errstr, "(coordinator.cpp) fork_controller() failed calling set_cpu(coordinator_pid,DEFAULT_PROCESSOR).\nExiting\n" );
                error_log.write( errstr );
                error_log.close( );
                printf( "%s", errstr );
                */
                return ERROR_FAILED;
            }

            // Note: may need to parameterize the relative priority for the current function's signature
            if( set_realtime_schedule_rel( pid, priority, 1 ) != ERROR_NONE ) {
                /*
                sprintf( errstr, "(coordinator.cpp) fork_controller() failed calling set_realtime_schedule_rel(controller_pid,controller_priority,1).\nExiting\n" );
                error_log.write( errstr );
                error_log.close( );
                printf( "%s", errstr );
                */
                return ERROR_FAILED;
            }

            //printf( "controller process priority: %d\n", priority );

            // execute the controller program
            // TODO : error handling
            execl( program.c_str(), program.c_str(), NULL );

            // exit on fail to exec.  Can only get here if execl fails.
            _exit( 1 );
        }

        return ERROR_NONE;
    }

    //-------------------------------------------------------------------------

    error_e create( const int& priority, thread_callback_fn callback ) {

        type = THREAD;

        pthread_attr_t attributes;
        struct sched_param param;

        pthread_attr_init( &attributes );

        // set an explicit schedule for the wakeup thread
        if( pthread_attr_setinheritsched( &attributes, PTHREAD_EXPLICIT_SCHED )  != 0 ) {
//            sprintf( errstr, "(coordinator.cpp) create_wakeup_thread() failed calling pthread_attr_setinheritsched(...)\n" );
//            error_log.write( errstr );
            return ERROR_FAILED;
        }

        // set schedule policy to round robin
        if( pthread_attr_setschedpolicy( &attributes, SCHED_RR ) != 0 ) {
//            sprintf( errstr, "(coordinator.cpp) create_wakeup_thread() failed calling pthread_attr_setschedpolicy(...)\n" );
//            error_log.write( errstr );
            return ERROR_FAILED;
        }

        param.sched_priority = priority;
        if( pthread_attr_setschedparam( &attributes, &param ) != 0 ) {
//            sprintf( errstr, "(coordinator.cpp) create_wakeup_thread() failed calling pthread_attr_setschedparam(...)\n" );
//            error_log.write( errstr );
            return ERROR_FAILED;
        }

        // create the wakeup thread
        if( pthread_create( &thread, &attributes, callback, NULL ) != 0 ) {
//            sprintf( errstr, "(coordinator.cpp) create_wakeup_thread() failed calling pthread_create(wakeup_thread,...)\n" );
//            error_log.write( errstr );
            return ERROR_FAILED;
        }

        // validation of the priority
        int policy;
        pthread_getschedparam( thread, &policy, &param );
//        printf( "wakeup priority: %d\n", param.sched_priority );

        pthread_attr_destroy( &attributes );

        return ERROR_NONE;
    }

    //-------------------------------------------------------------------------

    void suspend( void ) {

        if( type == PROCESS )
            kill( pid, SIGSTOP );

        suspended = true;
    }

    //-------------------------------------------------------------------------

    /// Continue the controller from the coordinator process
    void resume( void ) {

        suspended = false;

        if( type == PROCESS )
            kill( pid, SIGCONT );
    }

    //-------------------------------------------------------------------------

    void shutdown( void ) {

        if( type == PROCESS )
            kill( pid, SIGTERM );
            //kill( pid, SIGKILL );
    }

    //-------------------------------------------------------------------------
};

//-----------------------------------------------------------------------------

// Note: this is really a function of the scheduling system.
//yourtime_t dispatch(thread \in \{controller, simulator\}, yourtime_t max_duration, thd_status_t *status);
// TODO : determine approrpriate return type.  May create own time class.
void dispatch( thread_c& thread, const unsigned long long& max_duration, thread_status_e& status ) {

}

//-----------------------------------------------------------------------------

// gets the virtual time from a controlled process (or the "clock time" on a real robot)
// TODO : determine approrpriate return type.  May create own time class.
void get_time( thread_c& thread ) {

}

//-----------------------------------------------------------------------------

#endif // THREAD_H

