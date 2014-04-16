//#include <sstream>
#include <vector>
#include <sys/types.h>
#include <sys/mman.h>
#include <stdio.h>
#include <sys/wait.h>
#include <atomic>
#include <cstring>

#include "../cpu.h"
#include "../timer.h"
#include "../os.h"
#include "../types.h"
#include "../notification.h"
#include "../message.h"
#include "../processor.h"
#include "../timesink.h"
#include "../osthread.h"
#include "../dynamics.h"

#include "../channels.h"

#define DO_LOGGING
//#undef DO_LOGGING

//-----------------------------------------------------------------------------
static pid_t           coordinator_pid;
static int             coordinator_os_priority;
static cpu_speed_t     cpu_speed;
static cpu_id_t        cpu;
//-----------------------------------------------------------------------------
static int             CLIENT_OS_MAX_PRIORITY;
static int             CLIENT_OS_MIN_PRIORITY;
static thread_p        current_thread;

thread_p processor_thread;
thread_p prey_thread;
thread_p pred_thread;

//-----------------------------------------------------------------------------
std::atomic<int>        wakeup_enabled;
static notification_t   wakeup_note;
static int              wakeup_os_priority;
static int              wakeup_write_fd;
pid_t                   wakeup_pid;

//-----------------------------------------------------------------------------
static int             timer_os_priority;
static notification_t  timer_note;
static int             timer_write_fd;
timer_c                timer;
unsigned               caught_timer_events;
unsigned               actual_timer_events;
// Note: a little unreliability in actual_timer_events for each given step
// due to not using a mutex.  Can only be written to by handler, but can be
// read by coordinator main process, so no guarantees that any messages have 
// the correct info until the timer is shutdown.  Do not want to protect with 
// a mutex though as the overhead is too significant in terms of system calls.

#define MAX_TIMER_EVENTS 100
//#define TIMER_PERIOD_NSECS 1000000
#define TIMER_PERIOD_NSECS 10000000
//-----------------------------------------------------------------------------

//bool quit;
//int quit;
std::atomic<int>       quit;

//-----------------------------------------------------------------------------
//char errstr[ 256 ];
char spstr[512];

//-----------------------------------------------------------------------------
#define LOG_CAPACITY 1048576  // 1MB
//#define LOG_CAPACITY 1024
log_p info;
//-----------------------------------------------------------------------------

std::vector<int>        subscribed_fds;
fd_set                  pending_fds;

notification_t note;
// note.ts
// note.t
// note.source = {TIMER,WAKEUP,CLIENT}
// note.type = {IDLE,OPEN,CLOSE,READ,WRITE}

client_message_t msg;
client_message_buffer_c msgbuffer;

//-----------------------------------------------------------------------------
// Pthreads
pthread_t wakeup_thread;

//-----------------------------------------------------------------------------
// Threads
boost::shared_ptr<processor_c> processor;

boost::shared_ptr<dynamics_c> dynamics;

boost::shared_ptr<timesink_c> prey;
boost::shared_ptr<osthread_c> prey_controller;

boost::shared_ptr<timesink_c> pred;
boost::shared_ptr<osthread_c> pred_controller;
boost::shared_ptr<osthread_c> pred_planner;

//std::vector<osthread_p>  clients;

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool select( void ) {
//  __select( subscribed_fds, pending_fds );

  int max_fd = 0;

  assert( subscribed_fds.size() > 0 );

  FD_ZERO( &pending_fds );
  for( int i = 0; i < subscribed_fds.size(); i++ ) {
    max_fd = std::max( max_fd, subscribed_fds[i] );
    FD_SET( subscribed_fds[i], &pending_fds );
    //printf( "fd:%d, ", fds[i] );
  }
  //printf( "max_fd:%d\n", max_fd );
  if( select( max_fd + 1, &pending_fds, NULL, NULL, NULL ) == -1 ) {

    if( info ) {
      char buf[16];
      if( errno == EBADF ) 
        sprintf( buf, "EBADF" );
      else if( errno == EINTR ) 
        sprintf( buf, "EINTR" );
      else if( errno == EINVAL ) 
        sprintf( buf, "EINVAL" );
      else if( errno == ENOMEM ) 
        sprintf( buf, "ENOMEM" );
      else 
        sprintf( buf, "UNKNOWN" );

      sprintf( spstr, "ERROR : (coordinator.cpp) select() failed calling __select(...) : errno[%s]\n", buf );
      info->write( spstr );
    }

    return false;
  }
  return true;
}

//-----------------------------------------------------------------------------
void read_notifications( void ) {
  notification_t note;
  int fd;

  // check for a timer event.  If interrupted due to timer (but not blocked)
  // will reschedule into the run queue
  if( FD_ISSET( FD_TIMER_TO_COORDINATOR_READ_CHANNEL, &pending_fds ) != 0 ) {
    fd = FD_TIMER_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      if( info ) {
        sprintf( spstr, "ERROR : (coordinator.cpp) read_messages() failed calling __read(FD_TIMER_TO_COORDINATOR_READ_CHANNEL,...)\n" );
        info->write( spstr );
      }

    } else {
      if( info ) {
        sprintf( spstr, "read_notifications( note.source=TIMER, caught_timer_events=%d, actual_timer_events=%d", ++caught_timer_events, actual_timer_events );
        info->write( spstr );
      }

      //TODO : make a scheduler function for this case.  Want to offload logic.
      // reschedule any current thread
      if( current_thread ) {
        timesink_p sink = boost::dynamic_pointer_cast<timesink_c>(current_thread);
        if( current_thread->type() == thread_c::OSTHREAD ) {
          osthread_p osthread = boost::dynamic_pointer_cast<osthread_c>(sink);
          //if( !current_thread->enqueued )
            osthread->lower_priority();

          if( info ) {
            sprintf( spstr, ", current_thread=%s", current_thread->name );
            info->write( spstr );
          }
        }
        if( sink->owner ) {
          //reschedule
          // TODO: accurately update computation time and temporal time
          //if( !current_thread->enqueued ) 
            sink->owner->run_queue.push( current_thread );
 
          if( info ) {
            sprintf( spstr, ", owner=%s )\n", sink->owner->name );
            info->write( spstr );
          }
        } else {
          if( info ) {
            info->write( " )\n" );
          }
        }
      } else {
        if( info ) {
          info->write( "current_thread=NULL )\n" );
        }
      }

      if( caught_timer_events >= MAX_TIMER_EVENTS ) {
        //quit.store( 1, std::memory_order_relaxed  );
        //quit.store( 1, std::memory_order_seq_cst  );
        //quit = true;
        //quit++;
        if( info ) {
          info->flush();
        }
        kill( coordinator_pid, SIGTERM );
      }
    }
  }

  // - check for client specific notifications -
  // prey controller
  else if( FD_ISSET( FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL, &pending_fds ) != 0) {
    fd = FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      if( info ) {
        sprintf( spstr, "ERROR : (coordinator.cpp) read_messages() failed calling __read(FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL,...)\n" );
        info->write( spstr );
      }
      //printf( "%s\n", spstr );
    } else {
      if( info ) {
        sprintf( spstr, "read_notifications( note.source=CLIENT, client=prey_controller )\n" );
        info->write( spstr );
      }

      if( note.type == notification_t::CLOSE ) {
        prey_controller->invalidated = true;
      } else {
        prey_controller->message_queue.push( note );
      }

      // * Temporary *
      if( note.type == notification_t::READ ) {
        
      } else if( note.type == notification_t::READ ) {

      }
      // * End temporary *

    }
  }
  // predator planner
  else if( FD_ISSET( FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL, &pending_fds) != 0 ) {
    fd = FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      if( info ) {
        sprintf( spstr, "ERROR : (coordinator.cpp) read_messages() failed calling __read(FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL,...)\n" );
        info->write( spstr );
      }
      //printf( "%s\n", spstr );
    } else {
      if( info ) {
        sprintf( spstr, "read_notifications( note.source=CLIENT, client=pred_planner )\n" );
        info->write( spstr );
      }

      if( note.type == notification_t::CLOSE ) {
        pred_planner->invalidated = true;
      } else {
        pred_planner->message_queue.push( note );
      }
    }
  }
  // predator controller
  else if( FD_ISSET( FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL, &pending_fds ) != 0 ) {
    fd = FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      if( info ) {
        sprintf( spstr, "ERROR : (coordinator.cpp) read_messages() failed calling __read(FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL,...)\n" );
        info->write( spstr );
      }
      //printf( "%s\n", spstr );
    } else {

      if( info ) {
        sprintf( spstr, "read_notifications( note.source=CLIENT, client=pred_controller )\n" );
        info->write( spstr );
      }

      if( note.type == notification_t::CLOSE ) {
        pred_controller->invalidated = true;
      } else {
        pred_controller->message_queue.push( note );
      }
    }
  }

  // check block detection first.  If client blocked, needs to be put into 
  // blocking queue.  If falls through, might get put into run queue instead.
  else if( FD_ISSET( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL, &pending_fds) != 0 ) {
    fd = FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      if( info ) {
        sprintf( spstr, "ERROR : (coordinator.cpp) read_messages() failed calling __read(FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL,...)\n" );
        info->write( spstr );
       }
    } else {
      if( info ) {
        sprintf( spstr, "read_notifications( note.source=WAKEUP" );
        info->write( spstr );
      }

      //TODO : make a scheduler function for this case.  Want to offload logic.
      // reschedule any current thread
      if( current_thread ) {
        timesink_p sink = boost::dynamic_pointer_cast<timesink_c>(current_thread);
        if( current_thread->type() == thread_c::OSTHREAD ) {
          osthread_p osthread = boost::dynamic_pointer_cast<osthread_c>(sink);
          //if( !current_thread->enqueued ) 
            osthread->lower_priority();

          if( info ) {
            sprintf( spstr, ", current_thread=%s", current_thread->name );
            info->write( spstr );
          }
        }
        if( sink->owner ) {
          //reschedule
          // TODO: accurately update computation time and temporal time
          //if( !current_thread->enqueued ) 
            sink->owner->block_queue.push( current_thread );

          if( info ) {
            sprintf( spstr, ", owner=%s )\n", sink->owner->name );
            info->write( spstr );
          }
        } else {
          if( info ) {
            info->write( " )\n" );
          }
        }
      } else {
        if( info ) {
          info->write( ", current_thread=NULL )\n" );
        }
      }
///*
      if( caught_timer_events >= MAX_TIMER_EVENTS ) {
        //quit = true;
        //quit++;
        //quit.store( 1, std::memory_order_relaxed  );
        //quit.store( 1, std::memory_order_seq_cst  );
        if( info ) {
          info->flush();
        }
        kill( coordinator_pid, SIGTERM );
      }
//*/
      // reenable block detection notifications
      wakeup_enabled.store( 1, std::memory_order_seq_cst  );
    }
  }

}

//-----------------------------------------------------------------------------
/// API function to process notifications delivered to a thread.
/// @param thread the current thread.
/// @param runqueue the heap of threads ready to run for a given timesink.
/// @param waitqueue the heap of threads pending on I/O for a given timesink.
void process_notifications( const thread_p& caller, osthread_p& thread, thread_heap_c* runqueue, thread_heap_c* waitqueue ) {
  cycle_t reschedule_time;
  char spstr[512];

  // assert that the thread is valid 
  if( !thread ) return;

//#ifdef DEBUG
  if( info && thread ) {
    sprintf( spstr, "process_notifications( caller=%s, thread=%s, ", caller->name, thread->name );
    info->write( spstr );
    
    if( thread->owner ) {
      sprintf( spstr, "owner=%s, ... )\n", thread->owner->name );
    } else {
      sprintf( spstr, "owner=root, ... )\n" );
    }
    info->write( spstr );
  } else {
    sprintf( spstr, "process_notifications( caller=%s, thread=none, ... )\n", caller->name );
    info->write( spstr );
  }
//#endif

  // assert that the message queue is not empty
  if( thread->message_queue.empty() ) {
    runqueue->push( thread );
    return;
  }

  // peek at the message at the head of the queue
  notification_t msg = thread->message_queue.front();
  // remove the message from the queue
  thread->message_queue.pop();

  // if an idle message compute values and do updates
  if( msg.type == notification_t::IDLE ) {
    // compute when to reschedule the thread
    reschedule_time = thread->blocktill( msg.period );
    // update the temporal progress
    thread->temporal_progress += msg.period;
  } else if( msg.type == notification_t::OPEN ) {
    // encapsulated process has fully initialized
  } else if( msg.type == notification_t::CLOSE ) {
    // encapsulated process is dying, knows so, and had a chance to notify
    // * NO GUARANTEES THAT THIS NOTIFICATION IS RECEIVED *
  } else if( msg.type == notification_t::READ ) {
    // client needs to read data from shared memory. Please service the request.
    // * NOT SURE how to get forward this properly to dynamics in particular *
  } else if( msg.type == notification_t::WRITE ) {
    // client has written data to shared memory.  Please serve data to clients.
    // * NOT SURE how to forward this properly to dynamics so it can update, or
    //   to other clients that share the same owner/timesink *
  } 

//#ifdef DEBUG 
  if( info ) {
    // print the notification for debugging
    char note_type[8];
    char note_source[8];
    if( msg.type == notification_t::IDLE )
      sprintf( note_type, "IDLE" );
    else if( msg.type == notification_t::OPEN )
      sprintf( note_type, "OPEN" );
    else if( msg.type == notification_t::CLOSE )
      sprintf( note_type, "CLOSE" );
    else if( msg.type == notification_t::READ )
      sprintf( note_type, "READ" );
    else if( msg.type == notification_t::WRITE )
      sprintf( note_type, "WRITE" );

    if( msg.source == notification_t::TIMER )
      sprintf( note_source, "TIMER" );
    else if( msg.source == notification_t::WAKEUP )
      sprintf( note_source, "WAKEUP" );
    else if( msg.source == notification_t::CLIENT )
      sprintf( note_source, "CLIENT" );
  
    sprintf( spstr, "notification: type[%s], source[%s], ts[%llu], period[%llu] thread: computational_progress[%llu], temporal_progress[%llu]\n", note_type, note_source, msg.ts, msg.period, thread->computational_progress, thread->temporal_progress );
    info->write( spstr );
  }
//#endif

  // if an idle message, reschedule
  //if( msg.type == notification_t::IDLE && reschedule_time > thread->temporal_progress ) {
  if( msg.type == notification_t::IDLE ) {
    if( info ) {
      sprintf( spstr, "handling idle notification for %s\n", thread->name );
      info->write( spstr );
    }
/*
    // find the thread in the runqueue and remove it.
    for( unsigned i = 0; i < runqueue.size(); i++ ) {
      thread_p thd = runqueue.element(i);

      // if not the searched for thread continue searching
      if( thd != thread ) continue;

      // otherwise remove and stop searching
      runqueue.remove( i, thd );
      break;
    }
    // to be comprehensive, might be inclined to check waitqueue,
    // but logically, it cannot be in the waitqueue if it was running

    // update its progress (forecasting in this way will account for 'idle time')
*/
    thread->temporal_progress = reschedule_time;

    // add to the waitqueue
    waitqueue->push( thread );

    if( info ) {
      sprintf( spstr, "slept %s\n", thread->name );
      info->write( spstr );
    }
  } else {
    runqueue->push( thread );
  } 
  // TODO - Logic for sending message to other threads if need be
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void term_sighandler( int signum ) {
  printf( "coordinator received SIGTERM\n" );
  //quit = true;
  quit.store( 1, std::memory_order_seq_cst  );
}

//-----------------------------------------------------------------------------
void timer_sighandler( int signum, siginfo_t *si, void *data ) {
  //std::string err, eno;

  // grab a timestamp immediately
  timer_note.ts = generate_timestamp();

  // increment accounting
  actual_timer_events++;

  ssize_t bytes_written;
  // write the timer notification to the pipe
  if( __write( timer_write_fd, &timer_note, sizeof(notification_t), bytes_written ) != OS_ERROR_NONE ) {
    // TODO: restructure to reduce complexity here.
/*
    if( errno == EPIPE )
      eno = " errno: EPIPE";
    else if( errno == EAGAIN || errno == EWOULDBLOCK )
      eno = " errno: EWOULDBLOCK";
    else if( errno == EBADF )
      eno = " errno: EBADF";
    else if( errno == EDESTADDRREQ )
      eno = " errno: EDESTADDRREQ";
    else if( errno == EDQUOT )
      eno = " errno: EDQUOT";
    else if( errno == EFAULT )
      eno = " errno: EFAULT";
    else if( errno == EFBIG )
      eno = " errno: EFBIG";
    else if( errno == EINTR )
      eno = " errno: EINTR";
    else if( errno == EINVAL )
      eno = " errno: EINVAL";
    else if( errno == EIO )
      eno = " errno: EIO";
    else if( errno == ENOSPC )
      eno = " errno: ENOSPC";

    err = "(coordinator.cpp) timer_sighandler(...) failed making system call write(...)" + eno;
    //sprintf( spstr, "(coordinator.cpp) timer_sighandler(...) failed making system call write(...) %s\n", eno );
    printf( "%s\n", err );
    // TODO : determine if there is a need to recover
*/
  }
}

//-----------------------------------------------------------------------------
void* wakeup( void* ) {
  int allow_wakeup;

  while( 1 ) {
    allow_wakeup = wakeup_enabled.load( std::memory_order_seq_cst );
    if( allow_wakeup ) {
      wakeup_note.ts = generate_timestamp();

      // disable block detection notifications.  Coordinator will reenable.
      wakeup_enabled.store( 0, std::memory_order_seq_cst );
      if( write( wakeup_write_fd, &wakeup_note, sizeof(notification_t) ) == -1 ) {
        //printf( "(coordinator.cpp) wakeup() failed making system call write(...)\n" );
        // TODO : determine if there is a need to bomb or recover
      }
    }
  }
  return NULL;
}


//-----------------------------------------------------------------------------
#include <unistd.h>
#include <fcntl.h>
bool init_pipe( const int& read_fd, const int& write_fd, bool write_blocking = false ) {
  int flags;
  int fd[2];

  if( pipe( fd ) != 0 ) {
    return false;
  }
  flags = fcntl( fd[0], F_GETFL, 0 );
  fcntl( fd[0], F_SETFL, flags );
  flags = fcntl( fd[1], F_GETFL, 0 );
  if( write_blocking )
    fcntl( fd[1], F_SETFL, flags );
  else
    fcntl( fd[1], F_SETFL, flags | O_NONBLOCK );

  if( dup2( fd[0], read_fd ) == -1 ) {
    __close( fd[0] );
    __close( fd[1] );
    return false;
  }
  if( dup2( fd[1], write_fd ) == -1 ) {
    __close( read_fd );
    __close( fd[1] );
    return false;
  }
  return true;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_timer_pipe( void ) {
  return init_pipe( FD_TIMER_TO_COORDINATOR_READ_CHANNEL, FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_timer_pipe( void ) {
  __close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_wakeup_pipe( void ) {
  // should be write blocking or not?
  return init_pipe( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL, FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_wakeup_pipe( void ) {
  __close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_coordinator_to_preycontroller_pipe( void ) {
  return init_pipe( FD_COORDINATOR_TO_PREYCONTROLLER_READ_CHANNEL, FD_COORDINATOR_TO_PREYCONTROLLER_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_coordinator_to_preycontroller_pipe( void ) {
  __close( FD_COORDINATOR_TO_PREYCONTROLLER_READ_CHANNEL );
  __close( FD_COORDINATOR_TO_PREYCONTROLLER_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
bool init_preycontroller_to_coordinator_pipe( void ) {
  return init_pipe( FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL, FD_PREYCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_preycontroller_to_coordinator_pipe( void ) {
  __close( FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_PREYCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_coordinator_to_predplanner_pipe( void ) {
  return init_pipe( FD_COORDINATOR_TO_PREDPLANNER_READ_CHANNEL, FD_COORDINATOR_TO_PREDPLANNER_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_coordinator_to_predplanner_pipe( void ) {
  __close( FD_COORDINATOR_TO_PREDPLANNER_READ_CHANNEL );
  __close( FD_COORDINATOR_TO_PREDPLANNER_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
bool init_predplanner_to_coordinator_pipe( void ) {
  return init_pipe( FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL, FD_PREDPLANNER_TO_COORDINATOR_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_predplanner_to_coordinator_pipe( void ) {
  __close( FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_PREDPLANNER_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_coordinator_to_predcontroller_pipe( void ) {
  return init_pipe( FD_COORDINATOR_TO_PREDCONTROLLER_READ_CHANNEL, FD_COORDINATOR_TO_PREDCONTROLLER_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_coordinator_to_predcontroller_pipe( void ) {
  __close( FD_COORDINATOR_TO_PREDCONTROLLER_READ_CHANNEL );
  __close( FD_COORDINATOR_TO_PREDCONTROLLER_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
bool init_predcontroller_to_coordinator_pipe( void ) {
  return init_pipe( FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL, FD_PREDCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_predcontroller_to_coordinator_pipe( void ) {
  __close( FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_PREDCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_pipes( void ) {
  if( !init_timer_pipe() ) {
    return false;
  }

  if( !init_wakeup_pipe() ) {
    close_timer_pipe();
    return false;
  }

  if( !init_coordinator_to_preycontroller_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    return false;
  }
  if( !init_preycontroller_to_coordinator_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    close_coordinator_to_preycontroller_pipe();
    return false;
  }

  if( !init_coordinator_to_predplanner_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    close_coordinator_to_preycontroller_pipe();
    close_preycontroller_to_coordinator_pipe();
    return false;
  }
  if( !init_predplanner_to_coordinator_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    close_coordinator_to_preycontroller_pipe();
    close_preycontroller_to_coordinator_pipe();
    close_coordinator_to_predplanner_pipe();
    return false;
  }

  if( !init_coordinator_to_predcontroller_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    close_coordinator_to_preycontroller_pipe();
    close_preycontroller_to_coordinator_pipe();
    close_coordinator_to_predplanner_pipe();
    close_predplanner_to_coordinator_pipe();
    return false;
  }
  if( !init_predcontroller_to_coordinator_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    close_coordinator_to_preycontroller_pipe();
    close_preycontroller_to_coordinator_pipe();
    close_coordinator_to_predplanner_pipe();
    close_predplanner_to_coordinator_pipe();
    close_coordinator_to_predcontroller_pipe();
    return false;
  }

  subscribed_fds.push_back( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
  subscribed_fds.push_back( FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL );
  subscribed_fds.push_back( FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL );
  subscribed_fds.push_back( FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL );
  subscribed_fds.push_back( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );

  return true;
}

//-----------------------------------------------------------------------------
void close_pipes( void ) {
  close_timer_pipe();
  close_wakeup_pipe();
  close_coordinator_to_preycontroller_pipe();
  close_preycontroller_to_coordinator_pipe();
  close_coordinator_to_predplanner_pipe();
  close_predplanner_to_coordinator_pipe();
  close_coordinator_to_predcontroller_pipe();
  close_predcontroller_to_coordinator_pipe();
}

//-----------------------------------------------------------------------------
void init( int argc, char* argv[] ) {
  //quit = false;
  quit = 0;
  //quit.store( 0, std::memory_order_seq_cst  );
  //quit.store( 0, std::memory_order_relaxed  );

  actual_timer_events = 0;
  caught_timer_events = 0;

  // * set variables from constants *
  cpu = DEFAULT_CPU;

  // * install SIGTERM signal handler *
  struct sigaction action;
  memset( &action, 0, sizeof(struct sigaction) );
  action.sa_handler = term_sighandler;
  sigaction( SIGTERM, &action, NULL );

  // * open log *
#ifdef DO_LOGGING
  info = log_p( new log_c( "info.log", true ) );
  log_c::error_e log_err = info->allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling log_c::allocate(...).\nExiting\n" ); 
    printf( spstr );
    exit( 1 );
  }
#endif

  // * get the process identifier *
  coordinator_pid = getpid( );

  sprintf( spstr, "coordinator pid: %d\n", coordinator_pid );
  if( info ) info->write( spstr );
  printf( "%s", spstr );

  // * bind the process to a single cpu *
  if( cpu_c::bind( coordinator_pid, cpu ) != cpu_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling cpu_c::_bind(coordinator_pid,DEFAULT_CPU).\nExiting\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
    exit( 1 );
  }

  // * set the process to be scheduled with realtime policy and max priority *
  if( scheduler_c::set_realtime_policy( coordinator_pid, coordinator_os_priority ) != scheduler_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling schedule_set_realtime_max(coordinator_pid,coordinator_priority).\nExiting\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
    exit( 1 );
  }
  sprintf( spstr, "coordinator os priority: %d\n", coordinator_os_priority );
  if( info ) info->write( spstr );
  printf( "%s", spstr );

  // * determine if the OS supports high resolution timers *
  struct timespec res;
  clock_getres( CLOCK_MONOTONIC, &res );
  double clock_res = timespec_to_real( res );

  sprintf( spstr, "clock resolution (secs): %10.9f\n", clock_res );
  if( info ) info->write( spstr );
  printf( "%s", spstr );
 
  if( res.tv_sec != 0 && res.tv_nsec != 1 ) {
    sprintf( spstr, "(coordinator.cpp) init() failed.  The host operating system does not support high resolution timers.  Consult your system documentation on enabling this feature.\nExiting\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
    exit( 1 );
  }

  // * get the cpu speed *
  if( cpu_c::get_speed( cpu_speed, cpu ) != cpu_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling cpu_c::get_frequency(cpu_speed,cpu)\nExiting\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
    exit( 1 );
  }
  sprintf( spstr, "cpu speed(hz): %llu\n", cpu_speed );
  if( info ) info->write( spstr );
  printf( "%s", spstr );

  // * initialize pipes *
  if( !init_pipes() ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling init_pipes()\nExiting\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
    exit( 1 );
  }

  // * initialize shared memory *
  msgbuffer = client_message_buffer_c( CLIENT_MESSAGE_BUFFER_NAME, CLIENT_MESSAGE_BUFFER_MUTEX_NAME, true );
  if( msgbuffer.open( ) != client_message_buffer_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling actuator_msg_buffer_c.open(...,true)\nExiting\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
    close_pipes( );
    exit( 1 );
  }

  // * initialize block detection, i.e. wakeup * 
  wakeup_os_priority = coordinator_os_priority - 2;
  wakeup_enabled.store( 1 );
  // set up the wakeup overhead to minimize what changes inside
  wakeup_note.source = notification_t::WAKEUP;
  wakeup_write_fd = FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL;

  if( info ) info->flush();

  // lock into memory to prevent pagefaults.  do last before main loop
  mlockall( MCL_CURRENT );

///*
  if( scheduler_c::create( wakeup_thread, wakeup_os_priority, wakeup ) != scheduler_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling wakeup_thread.create(...,wakeup)\nExiting\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
    msgbuffer.close( );
    close_pipes( );
    exit( 1 );
  }
//*/
/*
  if( scheduler_c::get_priority( wakeup_thread, wakeup_os_priority ) != scheduler_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling get_priority(wakeup_thread,...)\nExiting\n" );
    info.write( spstr );
    msgbuffer.close( );
    close_pipes( );
    // kill the wakeup thread?
    exit(1);
  }
*/
  sprintf( spstr, "wakeup thread created... priority:%d\n", wakeup_os_priority );
  if( info ) info->write( spstr );
  printf( "%s", spstr );

  // * initialize clients *
  sprintf( spstr, "initializing clients\n" );
  if( info ) info->write( spstr );
  printf( "%s", spstr );

  scheduler_c::error_e schedulererr;

  double controller_floor_seconds = 0.001;
  double controller_ceiling_seconds = 0.01;
  double planner_floor_seconds = 0.01;
  double planner_ceiling_seconds = 0.5;
  std::string controller_seed = "1";

  char numbuf[16];

  sprintf(numbuf,"%llu",seconds_to_cycles(controller_floor_seconds, cpu_speed));
  std::string controller_floor = numbuf;
  sprintf(numbuf,"%llu",seconds_to_cycles(controller_ceiling_seconds, cpu_speed));
  std::string controller_ceiling = numbuf;
  sprintf(numbuf,"%llu",seconds_to_cycles(planner_floor_seconds, cpu_speed));
  std::string planner_floor = numbuf;
  sprintf(numbuf,"%llu",seconds_to_cycles(planner_ceiling_seconds, cpu_speed));
  std::string planner_ceiling = numbuf;

  CLIENT_OS_MAX_PRIORITY = coordinator_os_priority - 1;
  CLIENT_OS_MIN_PRIORITY = coordinator_os_priority - 3;
  int client_os_priority_step = CLIENT_OS_MAX_PRIORITY - CLIENT_OS_MIN_PRIORITY;

  log_c* pinfo = NULL;
  if( info ) pinfo = info.get();

  // - create a processor -
  processor = boost::shared_ptr<processor_c>( new processor_c( "processor_0" ) );
  timesink_p processor_timesink = boost::dynamic_pointer_cast<timesink_c>( processor );
  processor_thread = boost::dynamic_pointer_cast<thread_c>( processor );
  processor->info = pinfo;

  // - create dynamics process -
  dynamics = boost::shared_ptr<dynamics_c>( new dynamics_c( "dynamics", processor_timesink, cpu_speed ) );
  dynamics->info = pinfo;
  processor->run_queue.push( dynamics );

  // - create prey processes -
  prey = boost::shared_ptr<timesink_c>( new timesink_c( "prey", processor_timesink, scheduler_c::PROGRESS) );
  prey->info = pinfo;
  //prey->priority = 0;
  processor->run_queue.push( prey );
  prey_thread = boost::dynamic_pointer_cast<thread_c>(prey);

  prey_controller = boost::shared_ptr<osthread_c>( new osthread_c( "prey_controller", prey, &select, &read_notifications, &process_notifications, pinfo ) );
  prey_controller->_max_os_priority = CLIENT_OS_MAX_PRIORITY;
  prey_controller->_min_os_priority = CLIENT_OS_MIN_PRIORITY;
  prey_controller->_os_priority_step = client_os_priority_step;
  prey_controller->_cpu_speed = cpu_speed;
  prey_controller->priority = 0;
  prey_controller->desired_period = seconds_to_cycles( controller_ceiling_seconds, cpu_speed );
  prey->run_queue.push( prey_controller );

  schedulererr = scheduler_c::create( prey_controller, 3, DEFAULT_CPU, "client-process", "prey-controller", controller_seed.c_str(), controller_floor.c_str(), controller_ceiling.c_str() );
  //prey_controller->block();

  sprintf( spstr, "created prey-controller: pid[%d], _os_priority[%d], _os_priority_step[%d], _max_os_priority[%d], _min_os_priority[%d]\n", prey_controller->pid, prey_controller->_os_priority, prey_controller->_os_priority_step, prey_controller->_max_os_priority, prey_controller->_min_os_priority );
  if( info ) info->write( spstr );

  // - create predator processes -
  pred = boost::shared_ptr<timesink_c>( new timesink_c( "pred", processor_timesink, scheduler_c::PROGRESS) );
  pred->info = pinfo;
  //prey->priority = 0;
  processor->run_queue.push( pred );
  pred_thread = boost::dynamic_pointer_cast<thread_c>(pred);

  pred_controller = boost::shared_ptr<osthread_c>( new osthread_c( "pred_controller", pred, &select, &read_notifications, &process_notifications, pinfo ) );
  pred_controller->_max_os_priority = CLIENT_OS_MAX_PRIORITY;
  pred_controller->_min_os_priority = CLIENT_OS_MIN_PRIORITY;
  pred_controller->_os_priority_step = client_os_priority_step;
  pred_controller->_cpu_speed = cpu_speed;
  pred_controller->priority = 0;
  pred_controller->desired_period = seconds_to_cycles( controller_ceiling_seconds, cpu_speed );
  pred->run_queue.push( pred_controller );

  schedulererr = scheduler_c::create( pred_controller, 3, DEFAULT_CPU, "client-process", "pred-controller", controller_seed.c_str(), controller_floor.c_str(), controller_ceiling.c_str() );
  //prey_controller->block();

  sprintf( spstr, "created pred-controller: pid[%d], _os_priority[%d], _os_priority_step[%d], _max_os_priority[%d], _min_os_priority[%d]\n", pred_controller->pid, pred_controller->_os_priority, pred_controller->_os_priority_step, pred_controller->_max_os_priority, pred_controller->_min_os_priority );
  if( info ) info->write( spstr );

  // * initialize timer *
  // set up the timer handler overhead to minimize what changes inside
  timer_note.source = notification_t::TIMER;
  timer_write_fd = FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL;

  // create the timer
  if( timer.create( timer_sighandler, RTTIMER_SIGNAL ) != timer_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling timer.create(timer_sighandler,RTTIMER_SIGNAL)\nExiting\n" );
    pthread_cancel( wakeup_thread );
    int status;
    if( pred_controller ) {
      kill( pred_controller->pid, SIGTERM );
      waitpid( pred_controller->pid, &status, 0 );
    }

    if( prey_controller ) {
      kill( prey_controller->pid, SIGTERM );
      waitpid( prey_controller->pid, &status, 0 );
    }

    if( info ) info->write( spstr );
    printf( "%s", spstr );
    msgbuffer.close( );
    close_pipes( );
    exit( 1 );
  }

  // * initialize other resources *

  //clients.push_back( prey_controller );
  //clients.push_back( pred_controller );
  //clients.push_back( pred_planner );
} 

//-----------------------------------------------------------------------------
void shutdown( void ) {
  // unlock memory.  do before any other shutdown operations
  munlockall();

///*
  // kill the wakeup thread
  pthread_cancel( wakeup_thread );
  //sleep( 1 );
//*/

  // delete the timer
  timer.block();
  timer.destroy();

  // wait for close messages from all clients
/*
  bool clients_shutdown = false;
  while( !clients_shutdown ) {
    select();
    sleep( 5 );
  }
*/
  // wait for close messages from all clients
  int status;
///*
  if( pred_controller ) {
    kill( pred_controller->pid, SIGTERM );
    waitpid( pred_controller->pid, &status, 0 );
  }

  if( prey_controller ) {
    kill( prey_controller->pid, SIGTERM );
    waitpid( prey_controller->pid, &status, 0 );
  }
//*/
/*
  // kill the clients
  for( unsigned i = 0; i < clients.size(); i++ ) {
    kill( clients[i]->pid, SIGTERM );
    waitpid( clients[i]->pid, &status, 0 );
  }
*/
 /*
 while( !prey_controller->invalidated ) {
    // send notifications to clients
    select();
    //sleep( 1 );
    waitpid( prey_controller->pid, &status, 0 );
    //wait( &status );
  }
*/
  // close the shared buffer
  msgbuffer.close();

  // close the pipes
  close_pipes();

  // write any last statistics
  sprintf( spstr, "timer events: actual[%d], caught[%u]\n", actual_timer_events, caught_timer_events );
  if( info ) info->write( spstr );
  printf( spstr );

  sprintf( spstr, "coordinator shutdown\n" );
  if( info ) info->write( spstr );
  printf( spstr );

  // ensure the log gets written then destroyed
  if( info ) info->flush();
  if( info ) info->deallocate();
}

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] ) {
  // Due to realtime scheduling, et al, must have root access to run.
  if( geteuid() != 0 ) {
    sprintf( spstr, "This program requires root access.  Re-run with sudo.\nExiting\n" );
    printf( "%s", spstr );
    exit( 1 );
  }

  init( argc, argv );

  // last before main loop, arm the timer
  timer.arm( timer_c::PERIODIC, TIMER_PERIOD_NSECS );

  //while( !quit.load( std::memory_order_seq_cst ) ) {
  while( !quit.load( std::memory_order_relaxed ) ) {
  //while( !quit ) {
    scheduler_c::step_system( processor_thread, current_thread, processor->run_queue, processor->block_queue, info.get() );
  }

  shutdown();

  return 0;
}

