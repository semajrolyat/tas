#ifndef _GAZEBO_SHIP_DIRECTED_CONTROL_SAMPLER_H_
#define _GAZEBO_SHIP_DIRECTED_CONTROL_SAMPLER_H_

#include "ship.h"

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/control/Control.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

//-----------------------------------------------------------------------------
class directed_control_sampler_c : public ompl::control::SimpleDirectedControlSampler {
public:
  ship_c* ship;

  //---------------------------------------------------------------------------
  directed_control_sampler_c( const ompl::control::SpaceInformation *si, ship_c* _ship, unsigned int k = 1 ) :
    SimpleDirectedControlSampler(si, k) //, cs_(si->allocControlSampler()), numControlSamples_(k)
  {
    ship = _ship;
  }

  //---------------------------------------------------------------------------
  virtual ~directed_control_sampler_c( void ) {

  }

  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  virtual unsigned int getBestControl( ompl::control::Control *control, const ompl::base::State *source, ompl::base::State *dest, const ompl::control::Control *previous ) {

    const ompl::base::StateSpace *statespace = si_->getStateSpace().get();

    const double DT = ship->PLANNER_STEP_SIZE;
    const unsigned NUM_STEPS = 1;

    // generate state vector(s)
    std::vector<double> q0( ship_state_c::size() );
    from_state( statespace, source, q0 );
    std::vector<double> q1( ship_state_c::size() );
    from_state( statespace, dest, q1 );

    //std::cout << "q0:" << q0 << std::endl;
    //std::cout << "q1:" << q1 << std::endl;

    // normalize the destination state quaternion
    double qnorm = 0.0;
    for( unsigned i = 3; i < 7; i++ )
      qnorm += q1[i] * q1[i];
    qnorm = std::sqrt( qnorm );
    for( unsigned i = 3; i < 7; i++ )
      q1[i] /= qnorm;

    // generate differential vector
    std::vector<double> dq( ship_state_c::size() );
    for( unsigned i = 0; i < ship_state_c::size(); i++ )
      dq[i] = q1[i] - q0[i];

    // scale differential vector
    for( unsigned i = 0; i < ship_state_c::size(); i++ )
      dq[i];// /= DT;

    // update dq to update position
    dq[7] += dq[0];///DT;
    dq[8] += dq[1];///DT;
    dq[9] += dq[2];///DT;

    // update dq to update velocity components
    Ravelin::Quatd edot(dq[3], dq[4], dq[5], dq[6]);
    Ravelin::Quatd e(q0[3], q0[4], q0[5], q0[6]);
    Ravelin::Vector3d omega = Ravelin::Quatd::to_omega(e, edot);
    dq[10] += omega[0];///DT;
    dq[11] += omega[1];///DT;
    dq[12] += omega[2];///DT;

    // call inverse dynamics
    std::vector<double> u;
    ship->inv_dyn( q0, dq, u );

    //std::cout << "u:" << u << std::endl;

    // copy u to control
    double *ctl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    for( unsigned i = 0; i < ship_command_c::size(); i++ )
      ctl[i] = u[i];

    // copy source to dest to start integrating
    to_state( statespace, q0, dest );

    // integrate for the desired number of steps
    std::vector<double> deltaq;
    for( unsigned k = 0; k < NUM_STEPS; k++ ) {
      // get the ODE
      (*ship)(dest, control, deltaq, ship );

      // first, update the velocity components
      for( unsigned i = 7; i < ship_state_c::size(); i++ ) {
        q0[i] += DT*deltaq[i];
      }

      // now update the position components (first)
      q0[0] += DT*q0[7];
      q0[1] += DT*q0[8];
      q0[2] += DT*q0[9];

      // convert the angular velocity to a quaternion time derivative 
      Ravelin::Quatd e(q0[3], q0[4], q0[5], q0[6]);
      Ravelin::Vector3d omega(q0[10], q0[11], q0[12]);
      Ravelin::Quatd edot = Ravelin::Quatd::deriv(e, omega);

      // update the orientation components using the angular velocity
      q0[3] += DT*edot[0];
      q0[4] += DT*edot[1];
      q0[5] += DT*edot[2];
      q0[6] += DT*edot[3];

      // normalize quaternion
      double qnorm = 0.0;
      for( unsigned i = 3; i < 7; i++ )
        qnorm += q0[i]*q0[i];
      qnorm = std::sqrt(qnorm);
      for( unsigned i = 3; i < 7; i++ )
        q0[i] /= qnorm;

      // update dest
      to_state(statespace, q0, dest);

      //std::cout << "q0_dest[" << k << "]:" << q0 << std::endl;
    }

    // return 1
    return (unsigned) NUM_STEPS;
  }

};
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

#endif // _GAZEBO_SHIP_DIRECTED_CONTROL_SAMPLER_H_

