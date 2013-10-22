#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>

#include <pendulum/command.h>
#include <pendulum/state.h>
#include <pendulum/synch.h>

//#include <boost/shared_ptr.hpp>

typedef double Real;

#define PI 3.14159265359

//namespace gazebo
//{

//------------------------------------------------------------------------------

class ros_pendulum_controller_c : public gazebo::ModelPlugin {
private:

    gazebo::event::ConnectionPtr updateConnection;

    gazebo::physics::ModelPtr model;
    gazebo::physics::WorldPtr world;

    gazebo::physics::Joint_V joints;

    //gazebo::common::Time start_time, last_time, sim_time;

private: 
    ros::NodeHandle* node;
    ros::NodeHandle* node_state;

    // ROS Subscriber
    //ros::Subscriber sub;

    ros::Subscriber command_sub;
    ros::ServiceServer state_server;

    Real time, position, velocity, torque;

    bool controller_initialized;

public:
    //--------------------------------------------------------------------------
    // Contructors
    //--------------------------------------------------------------------------
    ros_pendulum_controller_c( ) { 
	controller_initialized = false;
	time = 0.0;
        position = 0.0;
        velocity = 0.0;
	torque = 0.0;
    }

    //--------------------------------------------------------------------------
    // Destructor
    //--------------------------------------------------------------------------
    virtual ~ros_pendulum_controller_c( ) {
        gazebo::event::Events::DisconnectWorldUpdateBegin( this->updateConnection );
        delete this->node_state;
	
        delete this->node;
    }
public:
    //--------------------------------------------------------------------------
    // Callbacks
    //--------------------------------------------------------------------------
    bool service_state( pendulum::state::Request &req, pendulum::state::Response &res ) {
/*
        std::cerr << "state requested: "
//			<< "time[" << req.time 
			<< "] position[" << position 
			<< "] velocity[" << velocity 
			<< "]" << std::endl;
*/
///*
	//res.time = time;
	//res.time = req.time;
	res.position = position;
	res.velocity = velocity;
//*/

        //ROS_INFO("request: time=%f, position=%f, velocity=%f", req.time, req.position, req.velocity);
        //ROS_INFO("response: torque=%f", res.torque);
        return true;
    }

    //--------------------------------------------------------------------------
///*
    void receive_command( const pendulum::command::ConstPtr& command ) {

	//ROS_INFO( "%10.9f, %7.6f", command->time, command->torque );
        //std::cerr << "command received: time[" << command->time << "] torque[" << command->torque << "]"  << std::endl;
	
	torque = command->torque;
        joints[0]->SetForce( 0, torque );

	if( !controller_initialized ) controller_initialized = true;
    }
//*/
/*
    void receive_command( const boost::shared_ptr<pendulum::command>& command ) {
	this->command.torque = command.torque;
    }
*/
private:
    //--------------------------------------------------------------------------
    // Utilities
    //--------------------------------------------------------------------------
    void update_state( void ) {

	if( controller_initialized ) {

            //sim_time = world->GetSimTime();

            //time = (sim_time - start_time).Double();
            position = this->joints[0]->GetAngle( 0 ).Radian( );
            velocity = this->joints[0]->GetVelocity( 0 );

            //last_time = sim_time;
        } else {
            //start_time = world->GetSimTime();
	}
    }
    //--------------------------------------------------------------------------
    // ModelPlugin Interface
    //--------------------------------------------------------------------------
protected:
    virtual void Load( gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf ) {
        int argc = 0;
        ros::init( argc, NULL, "pendulum_robot" );

        this->model = _model;
        this->world = _model->GetWorld();
        this->joints = this->model->GetJoints();

        this->node = new ros::NodeHandle( "pendulum" );
        this->node_state = new ros::NodeHandle( "pendulum" );

	state_server = this->node_state->advertiseService( "state", &ros_pendulum_controller_c::service_state, this );

        std::cerr << "Advertising service " << state_server.getService() << std::endl;

        command_sub = this->node->subscribe<pendulum::command>( "command", 1000, &ros_pendulum_controller_c::receive_command, this );


        //start_time = world->GetSimTime();
        //last_time = start_time;

        this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
            boost::bind( &ros_pendulum_controller_c::Update, this ) );

        std::cerr << "Pendulum Robot Loaded" << std::endl;
    }

    //--------------------------------------------------------------------------


    virtual void Update( ) {

        update_state( );

    }

    /*
    virtual void Reset( ) {

    }
    */

};

//------------------------------------------------------------------------------

GZ_REGISTER_MODEL_PLUGIN( ros_pendulum_controller_c )

//------------------------------------------------------------------------------

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
