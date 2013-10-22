#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <pendulum/rpc_command.h>
#include <boost/shared_ptr.hpp>

typedef double Real;

#define PI 3.14159265359

//namespace gazebo
//{

class ros_pendulum_controller_c : public gazebo::ModelPlugin {
private:

    gazebo::event::ConnectionPtr updateConnection;

    gazebo::physics::ModelPtr model;
    gazebo::physics::WorldPtr world;

    gazebo::physics::Joint_V joints;

    gazebo::common::Time start_time, last_time;
	
    // ROS Nodehandle
private: 
    ros::NodeHandle* node;

    // ROS Subscriber
    //ros::Subscriber sub;

    ros::ServiceClient client;
    //ros::ServiceServer server;

    Real time, position, velocity, torque;

public:
    ros_pendulum_controller_c( ) { 

    }
    virtual ~ros_pendulum_controller_c( ) {
        gazebo::event::Events::DisconnectWorldUpdateBegin( this->updateConnection );
        delete this->node;
    }
    //-----------------------------------------------------------------------------
    // ModelPlugin Interface
    //-----------------------------------------------------------------------------
/*
    void request_state( const boost::shared_ptr<pendulum::command>& state ) {
        std::cerr << "data received" << std::endl;
        //ROS_INFO( "subscriber got: [%f]", msg->data );
    }
*/
/*
    void issue_command( const pendulum::command request, const pendulum::command reply ) {
        std::cerr << "data received" << std::endl;
        //ROS_INFO( "subscriber got: [%f]", msg->data );
    }
*/
/*
    void ros_callback( const pendulum::command msg ) {
        std::cerr << "data received" << std::endl;
        //ROS_INFO( "subscriber got: [%f]", msg->data );
    }
*/
/*  
    void ros_callback( const pendulum::command::ConstPtr& msg ) {
        std::cerr << "data received" << std::endl;
        //ROS_INFO( "subscriber got: [%f]", msg->data );
    }
*/
protected:
    virtual void Load( gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf ) {
        // Start up ROS
        int argc = 0;
        ros::init( argc, NULL, "pendulum_robot" );
        //ros::init( argc, NULL, "pendulum" );

        this->model = _model;
        this->world = _model->GetWorld();
        this->joints = this->model->GetJoints();

        // ROS Nodehandle
        //this->node = new ros::NodeHandle( "~" );
        this->node = new ros::NodeHandle( "pendulum" );

        // ROS Subscriber
        //this->sub = this->node->subscribe<pendulum::command>( "ros_pendulum_standup_controller", 1, &ros_pendulum_controller_c::ros_callback, this );

        //this->client = this->node->serviceClient<pendulum::command>( "ros_pendulum_standup_controller" );

        this->client = this->node->serviceClient<pendulum::rpc_command>( "pendulum_standup_command" );
/*
	if( !client.isValid() ) {
	    std::cerr << "client not valid\n";
	    client.waitForExistence();
        }
*/
        //this->sub = this->node->subscribe<pendulum::command>( "ros_pendulum_standup_controller_state", 1, ros_pendulum_controller_c::request_state )

	//this->server = this->node->advertiseService( "ros_pendulum_standup_controller_command", &ros_pendulum_controller_c::issue_command );

        start_time = world->GetSimTime();
        last_time = start_time;

        this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
            boost::bind( &ros_pendulum_controller_c::Update, this ) );

        std::cerr << "Pendulum Loaded" << std::endl;
    }

    virtual void Update( ) {
/*
        common::Time sim_time = world->GetSimTime();

        Real time = (sim_time - start_time).Double();
        Real position = this->joints[0]->GetAngle( 0 ).Radian( );
        Real velocity = this->joints[0]->GetVelocity( 0 );
        Real torque = control( time, position, velocity );

        this->joints[0]->SetForce( 0, torque );

        last_time = sim_time;
*/
        gazebo::common::Time sim_time = world->GetSimTime();
        pendulum::rpc_command state;

        state.request.time = (sim_time - start_time).Double();
        state.request.position = this->joints[0]->GetAngle( 0 ).Radian( );
        state.request.velocity = this->joints[0]->GetVelocity( 0 );

	if( client.call(state) ) {
            Real torque = state.response.torque;	
            this->joints[0]->SetForce( 0, torque );
        }

        last_time = sim_time;
    }

    /*
    virtual void Reset( ) {

    }
    */



private:
    //-----------------------------------------------------------------------------
    // Trajectory Functions
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

};

GZ_REGISTER_MODEL_PLUGIN( ros_pendulum_controller_c )

//} // namespace gazebo



/*
#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace gazebo
{   
  class ROSModelPlugin : public ModelPlugin
  {

    public: ROSModelPlugin()
    {

      // Start up ROS
      std::string name = "ros_pendulum_plugin_node";
      int argc = 0;
      ros::init(argc, NULL, name);

    }
    public: ~ROSModelPlugin()
    {
      delete this->node;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr )
    {
      // Store the pointer to the model
      this->model = _parent;

      // ROS Nodehandle
      this->node = new ros::NodeHandle("~");

      // ROS Subscriber
      this->sub = this->node->subscribe<std_msgs::Float64>("x", 1000, &ROSModelPlugin::ROSCallback, this );

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&ROSModelPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      ros::spinOnce();
    }

    void ROSCallback(const std_msgs::Float64::ConstPtr& msg)
    {
      ROS_INFO("subscriber got: [%f]", msg->data);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* node;

    // ROS Subscriber
    ros::Subscriber sub;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSModelPlugin)
}
*/
