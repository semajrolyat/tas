/**
Based on ode::demo_plane2d
*/

#define dDOUBLE

// Test my Plane2D constraint.
// Uses ode-0.35 collision API.

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ode/ode.h>

#include <tas/tas.h>
#include <tas/time.h>
#include <tas/cpu.h>
#include <tas/schedule.h>
#include <tas/experiment.h>

//-----------------------------------------------------------------------------

timestamp_buffer_c timestamp_buffer;

char errstr[ 256 ];

#define PI 3.14159265359

//-----------------------------------------------------------------------------

#define drand48( )  ( (double) ( ( (double) rand() ) / ( (double) RAND_MAX) ) )

#define N_BODIES        1000
#define STAGE_SIZE      8.0  // in m

#define TIME_STEP       0.01
#define K_SPRING        10.0
#define K_DAMP          10.0

//-----------------------------------------------------------------------------

static dWorld   dyn_world;
static dBody    dyn_bodies[N_BODIES];
static dReal    bodies_sides[N_BODIES][3];

static dSpaceID coll_space_id;
static dJointID plane2d_joint_ids[N_BODIES];
static dJointGroup coll_contacts;

//-----------------------------------------------------------------------------

static void cb_start( ) {
	dAllocateODEDataForThread(dAllocateMaskAll);

    static float xyz[3] = { 0.5f * STAGE_SIZE, 0.5f * STAGE_SIZE, 0.65f * STAGE_SIZE };
    static float hpr[3] = { 90.0f, -90.0f, 0 };

}

//-----------------------------------------------------------------------------

static void cb_near_collision( void *data, dGeomID o1, dGeomID o2 ) {
    dBodyID b1 = dGeomGetBody( o1 );
    dBodyID b2 = dGeomGetBody( o2 );
    dContact contact;


    // exit without doing anything if the two bodies are static
    if( b1 == 0 && b2 == 0 )
        return;

    // exit without doing anything if the two bodies are connected by a joint
    if( b1 && b2 && dAreConnected( b1, b2 ) ) {
        /* MTRAP; */
        return;
    }

    contact.surface.mode = 0;
    contact.surface.mu = 0; // frictionless

    if( dCollide ( o1, o2, 1, &contact.geom, sizeof( dContactGeom ) ) ) {
        dJointID c = dJointCreateContact (dyn_world.id(),
                        coll_contacts.id (), &contact);
        dJointAttach( c, b1, b2 );
    }
}

//-----------------------------------------------------------------------------

static void track_to_pos( dBody &body, dJointID joint_id, dReal target_x, dReal target_y ) {
    dReal  curr_x = body.getPosition()[0];
    dReal  curr_y = body.getPosition()[1];

    dJointSetPlane2DXParam( joint_id, dParamVel, 1 * (target_x - curr_x) );
    dJointSetPlane2DYParam( joint_id, dParamVel, 1 * (target_y - curr_y) );
}

//-----------------------------------------------------------------------------

static void cb_sim_step( int pause ) {
    if( !pause ) {
        static dReal angle = 0;

        angle += REAL( 0.01 );

        track_to_pos( dyn_bodies[0], plane2d_joint_ids[0],
            dReal( STAGE_SIZE/2 + STAGE_SIZE/2.0 * cos (angle) ),
            dReal( STAGE_SIZE/2 + STAGE_SIZE/2.0 * sin (angle) ));

//        double u = drand48( ) * 2 * PI;
//        double v = drand48( ) * 2 * PI;

        double   f0 = 1.0;
        for (int b = 0; b < N_BODIES; b ++)
        {
            double u = drand48( ) * 2 * PI;
            double v = drand48( ) * 2 * PI;
            dyn_bodies[b].addForce (f0 * cos (u), f0 * sin (v), 0);
        }

        unsigned long long ts_before, ts_after;
        rdtscll( ts_before );

        const int n = 10;
        for( int i = 0; i < n; i ++ ) {
            dSpaceCollide( coll_space_id, 0, &cb_near_collision );
            dyn_world.step( dReal( TIME_STEP/n ) );
            coll_contacts.empty();
        }

        rdtscll( ts_after );

        timestamp_buffer.write( ts_after - ts_before );

    }

# if 1  /* [ */
    {
        // @@@ hack Plane2D constraint error reduction here:
        for( int b = 0; b < N_BODIES; b ++ ) {
            const dReal *rot = dBodyGetAngularVel( dyn_bodies[b].id() );
            const dReal *quat_ptr;
            dReal quat[4], quat_len;

            quat_ptr = dBodyGetQuaternion( dyn_bodies[b].id() );
            quat[0] = quat_ptr[0];
            quat[1] = 0;
            quat[2] = 0;
            quat[3] = quat_ptr[3];
            quat_len = sqrt( quat[0] * quat[0] + quat[3] * quat[3] );
            quat[0] /= quat_len;
            quat[3] /= quat_len;
            dBodySetQuaternion( dyn_bodies[b].id(), quat );
            dBodySetAngularVel( dyn_bodies[b].id(), 0, 0, rot[2] );
        }
    }
# endif  /* ] */


# if 0  /* [ */
    {
        // @@@ friction
        for (int b = 0; b < N_BODIES; b ++) {
            const dReal *vel = dBodyGetLinearVel( dyn_bodies[b].id() ),
                        *rot = dBodyGetAngularVel( dyn_bodies[b].id() );
            dReal s = 1.00;
            dReal t = 0.99;

            dBodySetLinearVel( dyn_bodies[b].id(), s*vel[0],s*vel[1],s*vel[2] );
            dBodySetAngularVel( dyn_bodies[b].id(),t*rot[0],t*rot[1],t*rot[2] );
        }
    }
# endif  /* ] */

}

//-----------------------------------------------------------------------------

void init( ) {

    unsigned long long cpu_hz;

    pid_t pid = getpid( );

    if( set_cpu( pid, DEFAULT_PROCESSOR ) != ERROR_NONE ) {
        sprintf( errstr, "init() failed calling set_cpu(0,0).\nExiting\n" );
        printf( "%s", errstr );
        exit( 1 );
    }
/*
    int priority;
    if( set_realtime_schedule_max( pid, priority ) != ERROR_NONE ) {
        sprintf( errstr, "(coordinator.cpp) init() failed calling set_realtime_schedule_max(0,priority).\nExiting\n" );
        printf( "%s", errstr );
        exit( 1 );
    }
    printf( "process priority: %d\n", priority );
*/

    //++++++++++++++++++++++++++++++++++++++++++++++++

    // Get the cpu speed
    if( get_cpu_frequency( cpu_hz, DEFAULT_PROCESSOR ) != CPUINFO_ERROR_NONE ) {
        sprintf( errstr, "init() failed calling read_cpuinfo()\nExiting\n" );
        printf( "%s", errstr );
        exit( 1 );
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++

    timestamp_buffer = timestamp_buffer_c( cpu_hz, "timestamps.dat" );
    if( timestamp_buffer.open( ) != ERROR_NONE ) {
        sprintf( errstr, "init() failed calling timestamp_buffer_c.open()\nExiting\n" );
        printf( "%s", errstr );
        exit( 1 );
    }
}

//-----------------------------------------------------------------------------

void shutdown( void ) {
    printf( "shutting down\n" );

    timestamp_buffer.close();
}

//-----------------------------------------------------------------------------

extern int main( int argc, char **argv ) {

    if( geteuid() != 0 ) {
        sprintf( errstr, "This program requires root access.  Re-run with sudo.\nExiting\n" );
        printf( "%s", errstr );
        exit( 1 );
    }

    init( );

    // +++++++++++++++++++++++++++++++++++++++

    int b;

    dInitODE2( 0 );

    // dynamic world

    dReal  cf_mixing;// = 1 / TIME_STEP * K_SPRING + K_DAMP;
    dReal  err_reduct;// = TIME_STEP * K_SPRING * cf_mixing;
    err_reduct = REAL( 0.5 );
    cf_mixing = REAL( 0.001 );
    dWorldSetERP( dyn_world.id (), err_reduct );
    dWorldSetCFM( dyn_world.id (), cf_mixing );
    dyn_world.setGravity( 0, 0.0, -1.0 );

    coll_space_id = dSimpleSpaceCreate( 0 );

    // dynamic bodies
    for( b = 0; b < N_BODIES; b ++ ) {
        int l = (int) ( 1 + sqrt ( (double) N_BODIES ) );
        dReal x = dReal( ( 0.5 + (b / l) ) / l * STAGE_SIZE );
        dReal y = dReal( ( 0.5 + (b % l) ) / l * STAGE_SIZE );
        dReal z = REAL( 1.0 ) + REAL( 0.1 ) * (dReal)drand48();

        bodies_sides[b][0] = dReal( 5 * (0.2 + 0.7*drand48()) / sqrt((double)N_BODIES) );
        bodies_sides[b][1] = dReal( 5 * (0.2 + 0.7*drand48()) / sqrt((double)N_BODIES) );
        bodies_sides[b][2] = z;

        dyn_bodies[b].create( dyn_world );
        dyn_bodies[b].setPosition( x, y, z/2 );
        dyn_bodies[b].setData( (void*) (size_t)b );
        dBodySetLinearVel( dyn_bodies[b].id (),
            dReal( 3 * (drand48 () - 0.5) ), 
            dReal( 3 * (drand48 () - 0.5) ), 0 );

        dMass m;
        m.setBox( 1, bodies_sides[b][0],bodies_sides[b][1],bodies_sides[b][2] );
        m.adjust( REAL(0.1) * bodies_sides[b][0] * bodies_sides[b][1] );
        dyn_bodies[b].setMass( &m );

        plane2d_joint_ids[b] = dJointCreatePlane2D( dyn_world.id (), 0 );
        dJointAttach( plane2d_joint_ids[b], dyn_bodies[b].id (), 0 );
    }

    dJointSetPlane2DXParam( plane2d_joint_ids[0], dParamFMax, 10 );
    dJointSetPlane2DYParam( plane2d_joint_ids[0], dParamFMax, 10 );


    // collision geoms and joints
    dCreatePlane( coll_space_id,  1, 0, 0, 0 );
    dCreatePlane( coll_space_id, -1, 0, 0, -STAGE_SIZE );
    dCreatePlane( coll_space_id,  0,  1, 0, 0 );
    dCreatePlane( coll_space_id,  0, -1, 0, -STAGE_SIZE );

    for (b = 0; b < N_BODIES; b ++) {
        dGeomID coll_box_id;
        coll_box_id = dCreateBox( coll_space_id, bodies_sides[b][0], bodies_sides[b][1], bodies_sides[b][2]);
        dGeomSetBody (coll_box_id, dyn_bodies[b].id ());
    }

    coll_contacts.create ();

    for( unsigned int i = 0; i < 10000; i++ ) {

        //printf( "iteration %d\n", i );

        cb_sim_step( false );

    }

    dCloseODE( );

    shutdown( );

    return 0;
}

//-----------------------------------------------------------------------------