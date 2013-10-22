#include "ros/ros.h"
#include "pendulum/state.h"
#include "pendulum/command.h"

typedef double Real;

#define PI 3.14159265369

//-----------------------------------------------------------------------------
/// The position trajectory function.  A cubic
Real position_trajectory( Real t, Real tfinal, Real q0, Real qdes ) {

    if( t > tfinal )
        return qdes;
    return -2 * (qdes - q0) / (tfinal*tfinal*tfinal) * (t*t*t) + 3 * (qdes - q0) / (tfinal*tfinal) * (t*t) + q0;
}

//-----------------------------------------------------------------------------
/// The velocity trajectory function.  The first derivative of the cubic
Real velocity_trajectory( Real t, Real tfinal, Real q0, Real qdes ) {

    if( t > tfinal )
        return 0.0;
    return -6 * (qdes - q0) / (tfinal*tfinal*tfinal) * (t*t) + 6 * (qdes - q0) / (tfinal*tfinal) * (t);
}

//-----------------------------------------------------------------------------
/// Control function for Standalone Controller
Real control( Real time, Real position, Real velocity ) {
    //const Real Kp = 1.0;
    //const Real Kv = 0.1;

    const Real Kp = 100.0;
    const Real Kv = 10.0;

    //const Real Kp = 50.0;
    //const Real Kv = 5.0;

    //const Real Kp = 20.0;
    //const Real Kv = 2.0;

    //const Real Kp = 10.0;
    //const Real Kv = 1.0;

    Real measured_position = position;
    Real measured_velocity = velocity;

    //Real desired_position = position_trajectory( time, 0.5, 0.0, PI );
    //Real desired_velocity = velocity_trajectory( time, 0.5, 0.0, PI );

    //Real desired_position = position_trajectory( time, 0.5, 0.0, PI );
    //Real desired_velocity = velocity_trajectory( time, 0.5, 0.0, PI );
    Real desired_position = position_trajectory( time, 20.0, 0.0, PI );
    Real desired_velocity = velocity_trajectory( time, 20.0, 0.0, PI );

    //Real desired_position = position_trajectory( time, 10.0, PI, 0.0 );
    //Real desired_velocity = velocity_trajectory( time, 10.0, PI, 0.0 );

    Real torque = Kp * ( desired_position - measured_position ) + Kv * ( desired_velocity - measured_velocity );

    return torque;
}
/*
//-----------------------------------------------------------------------------
bool control(pendulum::command::Request &req, pendulum::command::Response &res) {
  res.torque = control( req.time, req.position, req.velocity );

  ROS_INFO("request: time=%f, position=%f, velocity=%f", req.time, req.position, req.velocity);
  ROS_INFO("response: torque=%f", res.torque);
  return true;
}
*/
//-----------------------------------------------------------------------------

ros::ServiceClient state_client;
pendulum::state state;

ros::Publisher command_pub;
pendulum::command command;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pendulum_controller");
    //ros::init(argc, argv, "pendulum");
    ros::NodeHandle n("pendulum");
    ros::NodeHandle n_state("pendulum");

    command_pub = n.advertise<pendulum::command>( "command", 1000 );

    state_client = n_state.serviceClient<pendulum::state>( "state" );
    //state_client = n.serviceClient<pendulum::state>( "state" );
    //state_client = n.serviceClient<pendulum::state>( "state", true );
    if( !state_client.isValid( ) ) {
	ROS_INFO( "state_client not valid" );
        state_client.waitForExistence( );
    } 

    ros::Rate loop_rate( 25 );  
    //ros::Rate loop_rate( 10 );  

    ROS_INFO("Ready to standup pendulum.");

    command.time = 0.0;
    command.torque = 0.0;
    state.request.timestamp = 0.0;

    pendulum::state s;

    if( !state_client.isValid( ) ) {
	ROS_INFO( "state_client no longer valid" );
        state_client.waitForExistence( );
    } 

    int count = 0;

    ros::Time calibrate_time, start_time;

    while( ros::ok( ) ) {
    	calibrate_time = ros::Time::now( );

	if( calibrate_time.sec != 0 || calibrate_time.nsec != 0 )
	    break;
	
        ros::spinOnce( );		
    }

    start_time = ros::Time::now( );
    //ROS_INFO( "start_time: sec[%u] nsec[%u]", start_time.sec, start_time.nsec );
    ROS_INFO( "time, position, velocity, torque" );

    while( ros::ok( ) ) {

	//if( count == 1 ) break;

        if( state_client && state_client.exists( ) && state_client.isValid( ) ) {
            //ROS_INFO("Calling service");
            if( state_client.call( s ) ) {
                //ROS_INFO("Successful call");

    		ros::Time sim_time = ros::Time::now( );

    		//ROS_INFO( "sim_time: sec[%u] nsec[%u]", sim_time.sec, sim_time.nsec );

		//ros::Duration current_time = sim_time - start_time;

		//Real time = (Real) count / 1000.0;
		Real time = (sim_time - start_time).toSec();

	        command.torque = control( time, s.response.position, s.response.velocity );
  	        command.time = time;

		ROS_INFO( "%10.9f, %10.9f, %10.9f, %10.9f", time, s.response.position, s.response.velocity, command.torque );

                command_pub.publish( command );

	        //command.torque = control( state.response.time, state.response.position, state.response.velocity );
  	        //command.time = state.response.time;
            } else {
               ROS_INFO("Failed to call");
	    }
        } else {
            ROS_INFO("Persistent client is invalid");
        }
/*
        if( !state_client.isValid( ) ) {
	    ROS_INFO( "state_client not valid" );
	}

        if( !state_client.exists( ) )
            state_client.waitForExistence( );
	    
	if( state_client.call( s ) ) {
	//if( state_client.call( state ) ) {
	    ROS_INFO( "called state" );

	    command.torque = control( s.response.time, s.response.position, s.response.velocity );
  	    command.time = s.response.time;

	    //command.torque = control( state.response.time, state.response.position, state.response.velocity );
  	    //command.time = state.response.time;
        } else {
	    ROS_INFO( "calling state service failed" );
        }
        command_pub.publish( command );
*/
        ros::spinOnce( );
	count++;
    }

    ROS_INFO( "Controller shutting down" );

    return 0;
}
//-----------------------------------------------------------------------------

