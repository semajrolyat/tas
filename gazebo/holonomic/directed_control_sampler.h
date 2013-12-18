#ifndef _GAZEBO_SHIP_DIRECTED_CONTROL_SAMPLER_H_
#define _GAZEBO_SHIP_DIRECTED_CONTROL_SAMPLER_H_

//-----------------------------------------------------------------------------

#include "ship.h"

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/control/Control.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

//-----------------------------------------------------------------------------
/// Control sampler for the single ship scenario
class directed_control_sampler_c : public ompl::control::SimpleDirectedControlSampler {
public:
  ship_c* ship;

  //---------------------------------------------------------------------------
  // Contructors
  //---------------------------------------------------------------------------
  directed_control_sampler_c( const ompl::control::SpaceInformation *si, ship_c* _ship, unsigned int k = 1 ) :
    SimpleDirectedControlSampler(si, k) 
  {
    ship = _ship;
  }

  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~directed_control_sampler_c( void ) {

  }

  //---------------------------------------------------------------------------
  // Overloading base class
  //---------------------------------------------------------------------------
  virtual unsigned int getBestControl( ompl::control::Control *control, const ompl::base::State *source, ompl::base::State *dest, const ompl::control::Control *previous ) {

    const ompl::base::StateSpace *statespace = si_->getStateSpace().get();

    //const double DT = ship->PLANNER_STEP_SIZE;
    const double DT = ship->dtime;
    const unsigned NUM_STEPS = 1;

    // generate state vector(s)
    std::vector<double> q0( ship_state_c::size() );
    from_state( statespace, source, q0 );
    std::vector<double> q1( ship_state_c::size() );
    from_state( statespace, dest, q1 );

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
    ship_c::inv_dyn( q0, dq, u, ship );

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
    }

    // return 1
    return (unsigned) NUM_STEPS;
  }

};
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/// Control Sampler for Predator and Prey pursuit scenario
class pp_control_sampler_c : public ompl::control::SimpleDirectedControlSampler {
public:
  ship_c* pred;
  ship_c* prey;

  //---------------------------------------------------------------------------
  // Contructors
  //---------------------------------------------------------------------------
  pp_control_sampler_c( const ompl::control::SpaceInformation *si, ship_c* _pred, ship_c* _prey, unsigned int k = 1 ) :
    SimpleDirectedControlSampler(si, k) 
  {
    pred = _pred;
    prey = _prey;
  }

  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~pp_control_sampler_c( void ) {

  }

  //---------------------------------------------------------------------------
  // Overloading base class
  //---------------------------------------------------------------------------
  virtual unsigned int getBestControl( ompl::control::Control *control, const ompl::base::State *source, ompl::base::State *dest, const ompl::control::Control *previous ) {

    ompl::base::StateSpace *statespace = si_->getStateSpace().get();
    const double DT = pred->PLANNER_STEP_SIZE;
    const unsigned NUM_STEPS = 1;
    std::vector<double> q0_pred, q0_prey, q1_pred, q1_prey;

    // initialize state vectors from ompl state information
    pp_state_c q0( statespace, source );
    pp_state_c q1( statespace, dest );
    q0_pred = q0.pred_vector();
    q0_prey = q0.prey_vector();
    q1_pred = q1.pred_vector();
    q1_prey = q1.prey_vector();

    // normalize the destination state quaternion for the predator; the
    // prey's state cannot be determined using 'dest' (b/c we do not plan
    // for commands to get the prey from one state to another)
    ship_c::renormalize_state_quat(q1_pred);

    // generate differential vector
    std::vector<double> dq_pred( q1_pred.size() );
    std::vector<double> dq_prey( q1_prey.size() );
    for( unsigned i = 0; i < q1_pred.size(); i++ )
      dq_pred[i] = q1_pred[i] - q0_pred[i];

    // update dq to update position
    dq_pred[7] += dq_pred[0];
    dq_pred[8] += dq_pred[1];
    dq_pred[9] += dq_pred[2];

    // update dq to update velocity components
    Ravelin::Quatd edot(dq_pred[3], dq_pred[4], dq_pred[5], dq_pred[6]);
    Ravelin::Quatd e(q0_pred[3], q0_pred[4], q0_pred[5], q0_pred[6]);
    Ravelin::Vector3d omega = Ravelin::Quatd::to_omega(e, edot);
    dq_pred[10] += omega[0];
    dq_pred[11] += omega[1];
    dq_pred[12] += omega[2];

    // call inverse dynamics to generate command for predator
    std::vector<double> u_pred;
    ship_c::inv_dyn( q0_pred, dq_pred, u_pred, pred );

    // use prey policy to generate command for prey
    // NOTE: we *might* want to try multiple random commands generated using 
    // ship_c::prey_command(.) [we can effect that using multiple random inputs 
    // for the time input]. We can then integrate the prey state multiple times
    // and see which the predator can get closest to. This strategy would
    // only need to be tried if the planner has a really hard time planning. 
    double time_input = (double) rand();
    std::vector<double> u_prey;
    ship_c::compute_prey_command(q0_pred, q0_prey, u_prey, time_input, DT, &pred->space, prey);

    // copy u to control
    double *ctl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    for( unsigned i = 0; i < ship_command_c::size(); i++ )
      ctl[i] = u_pred[i];

    // integrate for the desired number of steps
    std::vector<double> deltaq;
    for( unsigned k = 0; k < NUM_STEPS; k++ ) {
      // get the ODEs
      ship_c::ode(q0_pred, u_pred, dq_pred, pred );
      ship_c::ode(q0_prey, u_prey, dq_prey, prey );

      // first, update the velocity components using Euler integration
      for( unsigned i = 7; i < ship_state_c::size(); i++ ) {
        q0_pred[i] += DT*dq_pred[i];
        q0_prey[i] += DT*dq_prey[i];
      }

      // now update the position components using Euler integration
      q0_pred[0] += DT*q0_pred[7];
      q0_pred[1] += DT*q0_pred[8];
      q0_pred[2] += DT*q0_pred[9];
      q0_prey[0] += DT*q0_prey[7];
      q0_prey[1] += DT*q0_prey[8];
      q0_prey[2] += DT*q0_prey[9];

      // convert the angular velocities to quaternion time derivatives 
      Ravelin::Quatd e_pred(q0_pred[3], q0_pred[4], q0_pred[5], q0_pred[6]);
      Ravelin::Vector3d omega_pred(q0_pred[10], q0_pred[11], q0_pred[12]);
      Ravelin::Quatd edot_pred = Ravelin::Quatd::deriv(e_pred, omega_pred);
      Ravelin::Quatd e_prey(q0_prey[3], q0_prey[4], q0_prey[5], q0_prey[6]);
      Ravelin::Vector3d omega_prey(q0_prey[10], q0_prey[11], q0_prey[12]);
      Ravelin::Quatd edot_prey = Ravelin::Quatd::deriv(e_prey, omega_prey);

      // update the orientation components using the angular velocity and
      // Euler integration
      q0_pred[3] += DT*edot_pred[0];
      q0_pred[4] += DT*edot_pred[1];
      q0_pred[5] += DT*edot_pred[2];
      q0_pred[6] += DT*edot_pred[3];
      q0_prey[3] += DT*edot_prey[0];
      q0_prey[4] += DT*edot_prey[1];
      q0_prey[5] += DT*edot_prey[2];
      q0_prey[6] += DT*edot_prey[3];

      // renormalize quaternions
      ship_c::renormalize_state_quat(q0_pred);
      ship_c::renormalize_state_quat(q0_prey);
    }

    // update destination state from q0_pred, q0_prey
    pp_state_c qf( q0_pred, q0_prey );
    qf.write_ompl_state( statespace, dest );

    // return number of steps taken 
    return (unsigned) NUM_STEPS;
  }

};
//-----------------------------------------------------------------------------

#endif // _GAZEBO_SHIP_DIRECTED_CONTROL_SAMPLER_H_

