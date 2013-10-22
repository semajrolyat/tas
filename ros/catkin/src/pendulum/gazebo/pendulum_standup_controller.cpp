#include "pendulum_standup_controller.h"

namespace gazebo
{

gazebo_pendulum_standup_controller_c::gazebo_pendulum_standup_controller_c( ) {

}

gazebo_pendulum_standup_controller_c::~gazebo_pendulum_standup_controller_c( ) {
    event::Events::DisconnectWorldUpdateBegin( this->updateConnection );
}

void gazebo_pendulum_standup_controller_c::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    this->model = _model;
    this->world = _model->GetWorld();
    this->joints = this->model->GetJoints();


    std::cerr << "Starting Simple Plugin" << std::endl;

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind( &gazebo_pendulum_standup_controller_c::Update, this ) );

    start_time = world->GetSimTime();
    last_time = start_time;

}

void gazebo_pendulum_standup_controller_c::Update( ) {
    common::Time sim_time = world->GetSimTime();

    Real time = (sim_time - start_time).Double();
    Real position = this->joints[0]->GetAngle( 0 ).Radian( );
    Real velocity = this->joints[0]->GetVelocity( 0 );
    Real torque = control( time, position, velocity );

    this->joints[0]->SetForce( 0, torque );

    last_time = sim_time;
}

/*
void gazebo_pendulum_standup_controller_c::Reset( ) {

}
*/

GZ_REGISTER_MODEL_PLUGIN( gazebo_pendulum_standup_controller_c )

} // namespace gazebo

