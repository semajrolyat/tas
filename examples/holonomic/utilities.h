#ifndef _GAZEBO_SHIP_UTILITIES_H_
#define _GAZEBO_SHIP_UTILITIES_H_

//-----------------------------------------------------------------------------

#include "constants.h"

#include <vector>
#include <iostream>
#include <string>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>

//-----------------------------------------------------------------------------
// Utilities
//-----------------------------------------------------------------------------
/// Write a vector of doubles to a stream
std::ostream& operator<<(std::ostream& ostr, const std::vector<double>& v);
//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const std::vector<unsigned>& v);
//-----------------------------------------------------------------------------
std::vector<double> operator*( const double& c, const std::vector<double>& v );
//-----------------------------------------------------------------------------
std::vector<double> operator*( const std::vector<double>& v, const double& c );
//-----------------------------------------------------------------------------
/// Determine the sign of a double
double sgn( const double& x );
//-----------------------------------------------------------------------------
/// Generate a random value between -1 and 1
double unit_rand( );

//-----------------------------------------------------------------------------
/// Copy a state vector to (a pointer to) ompl's state structure
void to_state( const ompl::base::StateSpace* space, const std::vector<double>& values, ompl::base::State* state, unsigned start_index, unsigned elements );

//-----------------------------------------------------------------------------
/// Copy from (a pointer to) ompl's state structure to a state vector
void from_state( const ompl::base::StateSpace* space, const ompl::base::State* state, std::vector<double>& values, unsigned start_index, unsigned elements );
//-----------------------------------------------------------------------------

#endif // _GAZEBO_SHIP_UTILITIES_H_

