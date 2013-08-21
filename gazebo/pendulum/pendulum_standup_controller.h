#ifndef _GAZEBO_PENDULUM_STANDUP_CONTROLLER_H_
#define _GAZEBO_PENDULUM_STANDUP_CONTROLLER_H_

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

typedef double Real;

#define PI 3.14159265359

namespace gazebo
{

class gazebo_pendulum_standup_controller_c : public ModelPlugin {
private:

    event::ConnectionPtr updateConnection;

    physics::ModelPtr model;
    physics::WorldPtr world;

    physics::Joint_V joints;

    common::Time start_time, last_time;

public:
    gazebo_pendulum_standup_controller_c( );
    virtual ~gazebo_pendulum_standup_controller_c( );
protected:
    //-----------------------------------------------------------------------------
    // ModelPlugin Interface
    //-----------------------------------------------------------------------------
    virtual void Load( physics::ModelPtr _model, sdf::ElementPtr _sdf );
    virtual void Update( );
    //virtual void Reset( );

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

} // namespace gazebo

#endif // _GAZEBO_PENDULUM_STANDUP_CONTROLLER_H_
