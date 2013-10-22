#include "ros/ros.h"
#include "pendulum/command.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_pendulum_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pendulum::command>("ros_pendulum_standup_controller");
  pendulum::command state;
  state.request.time = 0.0;
  state.request.position = 0.0;
  state.request.velocity = 0.0;
  if( client.call(state) )
  {
    ROS_INFO("torque: %f", state.response.torque );
  }
  else
  {
    ROS_ERROR("Failed to call service ros_pendulum_standup_controller");
    return 1;
  }

  return 0;
}
