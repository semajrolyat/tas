#include "ros/ros.h"
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

    Real measured_position = position;
    Real measured_velocity = velocity;

    //Real desired_position = position_trajectory( time, 0.5, 0.0, PI );
    //Real desired_velocity = velocity_trajectory( time, 0.5, 0.0, PI );

    Real desired_position = position_trajectory( time, 10.0, 0.0, PI );
    Real desired_velocity = velocity_trajectory( time, 10.0, 0.0, PI );

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
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_pendulum_standup_controller");
  ros::NodeHandle n;

  //ros::ServiceServer service = n.advertiseService("ros_pendulum_standup_controller", control);
  ROS_INFO("Ready to standup pendulum.");
  ros::spin();

  return 0;
}
