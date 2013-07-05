#ifndef _SCHEDULE_H_
#define _SCHEDULE_H_

#include <tas/tas.h>
#include <tas/error.h>

//-----------------------------------------------------------------------------

error_e set_cpu( const int& pid, const int& processor ) {
    // restrict the cpu set s.t. controller only runs on a single processor
    cpu_set_t cpuset_mask;
    // zero out the cpu set
    CPU_ZERO( &cpuset_mask );
    // set the cpu set s.t. controller only runs on 1 processor specified by processor
    CPU_SET( processor, &cpuset_mask );

    if ( sched_setaffinity( pid, sizeof(cpuset_mask), &cpuset_mask ) == -1 ) {
        // there was an error setting the affinity for the coordinator
        // NOTE: can check errno if this occurs
        //sprintf( strbuffer, "(coordinator.cpp) set_schedule() failed calling sched_setaffinity(coordinator_pid,...)\n" );
        //error_log.write( strbuffer );
        return ERROR_FAILED;
    }

//    // testing sanity check ... TO BE COMMENTED
//    int ret = sched_getaffinity( 0, sizeof(cpuset_mask), &cpuset_mask );
//    printf( " sched_getaffinity = %d, cpuset_mask = %08lx\n", sizeof(cpuset_mask), cpuset_mask );

    return ERROR_NONE;
}

//-----------------------------------------------------------------------------

// priority, processor
error_e set_realtime_schedule_max( const int& pid, int& priority ) {

    struct sched_param param;

    // set the scheduling policy and the priority where priority should be the
    // highest possible priority, i.e. min, for the round robin scheduler
    param.sched_priority = sched_get_priority_max( SCHED_RR );
    sched_setscheduler( pid, SCHED_RR, &param );

    // validate the scheduling policy
    int policy = sched_getscheduler( pid );
    if( policy != SCHED_RR ) {
        return ERROR_FAILED;
    }

    // validate the priority
    sched_getparam( pid, &param );
    priority = param.sched_priority;

    return ERROR_NONE;
}

//-----------------------------------------------------------------------------

error_e set_realtime_schedule_rel( const int& pid, int& priority, const int& priority_offset ) {

    struct sched_param param;

    param.sched_priority = sched_get_priority_max( SCHED_RR ) - priority_offset;
    sched_setscheduler( pid, SCHED_RR, &param );

    // query the current policy
    int policy = sched_getscheduler( pid );
    if( policy == -1 ) {
        return ERROR_FAILED;
    }

    // validate the priority
    sched_getparam( pid, &param );
    priority = param.sched_priority;

    return ERROR_NONE;
}

//-----------------------------------------------------------------------------

#endif // _SCHEDULE_H_
