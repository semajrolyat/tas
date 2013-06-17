/******************************************************************************
* PD Controller for balancing an inverted pendulum
* to be used with pendulum.xml in the <Moby>/example/inverted-pendulum
*
* TODO
* 1. Limit on Torque <Reconsideration of determined probably not valuable>
*      The torque maxes out at around 0.04 in the current example
* 2. Delay in Control Inputs
* 3. Assymetrical Maximum Torque <probably not valuable as in 1.>
*
******************************************************************************/

#include <ostream>
#include <sstream>
#include <Moby/Simulator.h>
#include <Moby/RCArticulatedBody.h>

//-----------------------------------------------------------------------------

using namespace Moby;
using boost::shared_ptr;
using boost::dynamic_pointer_cast;

//-----------------------------------------------------------------------------

#define PI 3.14159265359 

//-----------------------------------------------------------------------------

Real prev_error;

std::ofstream file_pivot;

extern boost::shared_ptr<Simulator> sim;

DynamicBodyPtr robot;

//-----------------------------------------------------------------------------
// Trajectory Functions
//-----------------------------------------------------------------------------

/// The position trajectory function.  A cubic
Real position_trajectory( Real t, Real tfinal, Real q0, Real qdes ) {
  if( t > tfinal )
    return qdes;
  return -2 * (qdes - q0) / (tfinal*tfinal*tfinal) * t*t*t + 3 * (qdes - q0) / (tfinal*tfinal) * t*t + q0;
}

//-----------------------------------------------------------------------------

/// The velocity trajectory function.  The derivative of the cubic
Real velocity_trajectory( Real t, Real tfinal, Real q0, Real qdes ) {
  if( t > tfinal )
    return 0.0;
  return -6 * (qdes - q0) / (tfinal*tfinal*tfinal) * t*t + 6 * (qdes - q0) / (tfinal*tfinal) * t;
}

//-----------------------------------------------------------------------------
// Control Functions
//-----------------------------------------------------------------------------

/// Controls the pendulum
void control_PD( RCArticulatedBodyPtr pendulum, Real time ) {

  //Real error, derror;

  //const Real Kp = 0.5;	//1
  //const Real Kp = 1.0;	//2
  //const Real Kp = 2.0;	//3
  //const Real Kp = 4.0;	//4
  //const Real Kp = 8.0;	//5
  //const Real Kp = 16.0;	//6

  //const Real Kp = 700.0;
  //const Real Kv = 8.0;

  //const Real Kv = 0.1;		//1
  //const Real Kv = 0.2;		//2
  //const Real Kv = 0.5;		//3
  //const Real Kv = 1.0;		//4
  //const Real Kv = 2.0;		//5

  const Real Kp = 1.0;
  const Real Kv = 0.1;
/*
  const Real Kp = 4.0;
  const Real Kv = 0.4;
*/
/*
  const Real Kp = 0.1;
  const Real Kv = 0.01;
*/
  //Real theta, dtheta;

  JointPtr pivot = pendulum->find_joint( "pivot" );
  if( !pivot )
      std::cerr << "Could not find pivot joint\n";

  Real measured_position = pivot->q[0];
  Real measured_velocity = pivot->qd[0];

  // 3 o'clock
  //Real desired_position = position_trajectory( time, 10.0, 0.0, PI/2.0 );
  //Real desired_velocity = velocity_trajectory( time, 10.0, 0.0, PI/2.0 );

  // 6 o'clock
  Real desired_position = position_trajectory( time, 0.5, 0.0, PI );
  Real desired_velocity = velocity_trajectory( time, 0.5, 0.0, PI );


  Real torque = Kp * ( desired_position - measured_position ) + Kv * ( desired_velocity - measured_velocity );

  VectorN tau( 1 );
  tau[0] = torque;
  pivot->add_force( tau );

  //file_pivot << time << "," << measured_position << "," << desired_position << "," << measured_velocity << "," << desired_velocity << "," << torque << "\n";

  //if( time > 10.0 )
  //  std::cout << "10.0 secs have passed.\n";
}

//-----------------------------------------------------------------------------
/*
/// The main control loop
void control( DynamicBodyPtr pendulum, Real time, void* data ) {

  control_PD( dynamic_pointer_cast<RCArticulatedBody>(pendulum), time );
}
*/

void control( ) {

  Real time = sim->current_time;

  control_PD( dynamic_pointer_cast<RCArticulatedBody>(robot), time );
}


//-----------------------------------------------------------------------------
// Plugin Interface
//-----------------------------------------------------------------------------

// plugin must be "extern C"
extern "C" {

/// plugin entry point
void init( void* seperator, const std::map<std::string, BasePtr>& read_map, Real time ) {

  if( read_map.find("simulator") == read_map.end() )
    throw std::runtime_error( "inverted-pendulum-pd-controller.cpp:init()- unable to find simulator!" );
  // get a reference to the EventDrivenSimulator instance
  sim = dynamic_pointer_cast<Simulator>( read_map.find( "simulator" )->second );

  // find the pendulum
  if( read_map.find("pendulum") == read_map.end() )
    throw std::runtime_error( "inverted-pendulum-pd-controller.cpp:init()- unable to find pendulum!" );
  DynamicBodyPtr pendulum = dynamic_pointer_cast<DynamicBody>( read_map.find("pendulum")->second );
  if( !pendulum )
    throw std::runtime_error( "inverted-pendulum-pd-controller.cpp:init()- unable to cast pendulum to type DynamicBody" );

  robot = pendulum;

  // setup the controller
  // pendulum->controller = control;

  //file_pivot.open( "pivot.dat" );

  //printf( "inverted-pendulum-pd-controller initialized\n" );
}
/*
bool enlist( const std::map<std::string, BasePtr>& read_map, const std::string& body_id ) {

}
*/
} // end extern C

//-----------------------------------------------------------------------------
