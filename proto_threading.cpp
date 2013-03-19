/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

Very simple threading program to validate operations
-----------------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <signal.h>
#include <stdexcept>
#include <sys/time.h>
#include <sys/resource.h>
#include <errno.h>

#include <fstream>
//-----------------------------------------------------------------------------

#define _DEBUG_TO_STDOUT 1

//-----------------------------------------------------------------------------

static int sim_pid;
static int sim_priority;

static int controller_pid;

std::ofstream controller_log;

//-----------------------------------------------------------------------------

void fork_controller() {

    controller_pid = fork();
    if (controller_pid < 0) {
        throw std::runtime_error( "Failed to fork controller." ) ;
    }
    if (controller_pid == 0) {
        controller_pid = getpid();

        int priority_result = setpriority( PRIO_PROCESS, controller_pid, sim_priority + 1 );
        int controller_priority = getpriority( PRIO_PROCESS, controller_pid );

        controller_log.exceptions( std::ofstream::failbit );

        try {
            controller_log.open("controller.log");
        } catch (std::ofstream::failure ex) {
            printf( "FAILED TO OPEN STREAM.\n" );  // Likely won't print to console
        }

        if( priority_result < 0 ) {
            controller_log << "ERROR: Failed to set priority: " << priority_result << "\n";
        } else {
            controller_log << "Controller PID: " << controller_pid << "\n";
            controller_log << "Controller Priority: " << controller_priority << "\n";
        }

        // execute the controller program
        execl( "controller", "contoller", 0 );

        // exit on fail to exec
        _exit(1);
    }
}

//-----------------------------------------------------------------------------


//#define ITS_TO_WAIT             850000
#define ITS_TO_WAIT             8500000

pthread_t monitor_thread;
pthread_attr_t monitor_thread_attr;

pthread_t sim_thread;
pthread_attr_t sim_thread_attr;


void* monitor( void* arg ) {

    // time suck
    double x = 1.1;
    for( int i = 0; i < ITS_TO_WAIT; i++ ) {
        x *= x;
    }
}


//-----------------------------------------------------------------------------
// Simulator Entry Point
//-----------------------------------------------------------------------------
int main( int argc, char* argv[] ) {

    int result;

    controller_pid = 0;

    sim_pid = getpid();

    cpu_set_t cpuset_mask;
    // zero out the cpu set
    CPU_ZERO( &cpuset_mask );
    // set the cpu set s.t. controller only runs on 1 processor specified by DEFAULT_CONTROLLER_PROCESSOR
    CPU_SET( 0, &cpuset_mask );
    if ( sched_setaffinity( sim_pid, sizeof(cpuset_mask), &cpuset_mask ) == -1 ) {
        printf( "ERROR: Failed to set affinity for sim process.\n" );
        _exit( EXIT_FAILURE );
    }

    /*

    struct sched_param sim_params;
    sim_thread = pthread_self();
    //sim_params.sched_priority = 0;
    result = pthread_setschedparam( sim_thread, SCHED_RR, &sim_params );
    if( result != 0 ) {
        switch( errno ) {
        case EINVAL:
            printf( "errno: EINVAL\n" );
            break;
        case EPERM:
            printf( "errno: EPERM\n" );
            break;
        case ESRCH:
            printf( "errno: ESRCH\n" );
            break;
        default:
            printf( "errno: Unenumerated\n" );
            break;
        }

        _exit( EXIT_FAILURE );
    }
    */
    /*
    int sched_policy = sched_getscheduler( 0 );
    if ( sched_policy == -1 ) {
        printf( "ERROR: Failed to get scheduler policy for controller process.\n" );
        _exit( EXIT_FAILURE );
    } else {
        printf( "Sim Scheduling Policy: %d\n", sched_policy );

        struct sched_param params;
        params.sched_priority = 1;
        result = sched_setscheduler( 0, SCHED_RR, &params );
        if( result != 0 ) {
            switch( errno ) {
            case EINVAL:
                printf( "errno: EINVAL\n" );
                break;
            case EPERM:
                printf( "errno: EPERM\n" );
                break;
            case ESRCH:
                printf( "errno: ESRCH\n" );
                break;
            default:
                printf( "errno: Unenumerated\n" );
                break;
            }

            // error
            _exit( EXIT_FAILURE );
        }
    }
    */
    sim_priority = getpriority( PRIO_PROCESS, sim_pid );


    //fork_controller();

    result = pthread_attr_init( &monitor_thread_attr );
    if( result != 0 ) {
        // error
        _exit( EXIT_FAILURE );
    }

    struct sched_param monitor_sched_param;

    //monitor_sched_param.sched_priority = sched_getscheduler( pthread_self() ) + 2;

    result = pthread_attr_setinheritsched( &monitor_thread_attr, PTHREAD_EXPLICIT_SCHED );
    if( result != 0 ) {
        // error
        _exit( EXIT_FAILURE );
    }

    result = pthread_attr_setschedpolicy( &monitor_thread_attr, SCHED_RR );
    if( result != 0 ) {
        // error
        _exit( EXIT_FAILURE );
    }

    monitor_sched_param.sched_priority = sim_priority + 2;
    printf( "App Priority: %d\n", sim_priority );
    printf( "Monitor Priority: %d\n", monitor_sched_param.sched_priority );
    result = pthread_attr_setschedparam( &monitor_thread_attr, &monitor_sched_param );
    if( result != 0 ) {
        // error
        _exit( EXIT_FAILURE );
    }

    result = pthread_create( &monitor_thread, &monitor_thread_attr, &monitor, NULL );
    if( result != 0 ) {
        switch( result ) {
        case EAGAIN:
            printf( "result: EAGAIN\n" );
            break;
        case EPERM:
            printf( "result: EPERM\n" );
            break;
        case EINVAL:
            printf( "result: EINVAL\n" );
            break;
        default:
            printf( "result: Unenumerated %d\n", result );
            break;
        }
        // error
        perror( "Error pthread_create" );
        _exit( EXIT_FAILURE );
    }

    void* return_value;
    result = pthread_join( monitor_thread, &return_value );
    if( result != 0 ) {
        // error
        _exit( EXIT_FAILURE );
    }

    return 0;
}


//-----------------------------------------------------------------------------
// Below is the sample pthread program from man pthread_create(3)
//-----------------------------------------------------------------------------

/*
#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>

#define handle_error_en(en, msg) \
       do { errno = en; perror(msg); exit(EXIT_FAILURE); } while (0)

#define handle_error(msg) \
       do { perror(msg); exit(EXIT_FAILURE); } while (0)

struct thread_info {    // Used as argument to thread_start()
   pthread_t thread_id;        // ID returned by pthread_create()
   int       thread_num;       // Application-defined thread #
   char     *argv_string;      // From command-line argument
};

// Thread start function: display address near top of our stack,
//  and return upper-cased copy of argv_string

static void *
thread_start(void *arg)
{
   struct thread_info *tinfo = (struct thread_info *) arg;
   char *uargv, *p;

   printf("Thread %d: top of stack near %p; argv_string=%s\n",
           tinfo->thread_num, &p, tinfo->argv_string);

   uargv = strdup(tinfo->argv_string);
   if (uargv == NULL)
       handle_error("strdup");

   for (p = uargv; *p != '\0'; p++)
       *p = toupper(*p);

   return uargv;
}

int
main(int argc, char *argv[])
{
   int s, tnum, opt, num_threads;
   struct thread_info *tinfo;
   pthread_attr_t attr;
   int stack_size;
   void *res;

   // The "-s" option specifies a stack size for our threads

   stack_size = -1;
   while ((opt = getopt(argc, argv, "s:")) != -1) {
       switch (opt) {
       case 's':
           stack_size = strtoul(optarg, NULL, 0);
           break;

       default:
           fprintf(stderr, "Usage: %s [-s stack-size] arg...\n",
                   argv[0]);
           exit(EXIT_FAILURE);
       }

   }

   num_threads = argc - optind;

   // Initialize thread creation attributes

   s = pthread_attr_init(&attr);
   if (s != 0)
       handle_error_en(s, "pthread_attr_init");

   if (stack_size > 0) {
       s = pthread_attr_setstacksize(&attr, stack_size);
       if (s != 0)
           handle_error_en(s, "pthread_attr_setstacksize");
   }

   // Allocate memory for pthread_create() arguments

   tinfo = (thread_info*)calloc(num_threads, sizeof(struct thread_info));
   if (tinfo == NULL)
       handle_error("calloc");

   // Create one thread for each command-line argument

   for (tnum = 0; tnum < num_threads; tnum++) {
       tinfo[tnum].thread_num = tnum + 1;
       tinfo[tnum].argv_string = argv[optind + tnum];

       // The pthread_create() call stores the thread ID into
       // corresponding element of tinfo[]

       s = pthread_create(&tinfo[tnum].thread_id, &attr,
                          &thread_start, &tinfo[tnum]);
       if (s != 0)
           handle_error_en(s, "pthread_create");
   }

   // Destroy the thread attributes object, since it is no
   // longer needed

   s = pthread_attr_destroy(&attr);
   if (s != 0)
       handle_error_en(s, "pthread_attr_destroy");

   // Now join with each thread, and display its returned value

   for (tnum = 0; tnum < num_threads; tnum++) {
       s = pthread_join(tinfo[tnum].thread_id, &res);
       if (s != 0)
           handle_error_en(s, "pthread_join");

       printf("Joined with thread %d; returned value was %s\n",
               tinfo[tnum].thread_num, (char *) res);
       free(res);      // Free memory allocated by thread
   }

   free(tinfo);
   exit(EXIT_SUCCESS);
}
*/

