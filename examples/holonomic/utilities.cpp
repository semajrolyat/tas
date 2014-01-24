#include "utilities.h"

//-----------------------------------------------------------------------------
// Utilities
//-----------------------------------------------------------------------------
/// Write a vector of doubles to a stream
std::ostream& operator<<(std::ostream& ostr, const std::vector<double>& v) {
  for( unsigned i = 0; i < v.size(); i++ ) {
    if( i > 0 )
      ostr << ",";
    ostr << v[i];
  }
  return ostr;
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const std::vector<unsigned>& v) {
  for( unsigned i = 0; i < v.size(); i++ ) {
    if( i > 0 )
      ostr << ",";
    ostr << v[i];
  }
  return ostr;
}

//-----------------------------------------------------------------------------
std::vector<double> operator*( const double& c, const std::vector<double>& v ) {
  std::vector<double> x( v.size() );
  for( unsigned i = 0; i < v.size(); i++ )
    x = c * v;
  return x;
}

//-----------------------------------------------------------------------------
std::vector<double> operator*( const std::vector<double>& v, const double& c ) {
  std::vector<double> x( v.size() );
  for( unsigned i = 0; i < v.size(); i++ )
    x = c * v;
  return x;
}

//-----------------------------------------------------------------------------
/// Determine the sign of a double
double sgn( const double& x ) {
  if (x > 0.0)
    return 1.0;
  else if (x < 0.0)
    return -1.0;
  else
    return 0.0;
}

//-----------------------------------------------------------------------------
/// Generate a random value between -1 and 1
double unit_rand( )
{
  return (double) rand() / RAND_MAX * 2.0 - 1.0;
}

//-----------------------------------------------------------------------------
/// Copy a state vector to (a pointer to) ompl's state structure
void to_state( const ompl::base::StateSpace* space, const std::vector<double>& values, ompl::base::State* state, unsigned start_index = 0, unsigned elements = 13 ) {
  for( unsigned i = start_index; i < elements; i++ )
    *space->getValueAddressAtIndex(state, i) = values[i];
}


//-----------------------------------------------------------------------------
/// Copy from (a pointer to) ompl's state structure to a state vector
void from_state( const ompl::base::StateSpace* space, const ompl::base::State* state, std::vector<double>& values, unsigned start_index = 0, unsigned elements = 13 ) {
  for( unsigned i = start_index; i < elements; i++ )
    values[i] = *space->getValueAddressAtIndex(state, i);
}

//-----------------------------------------------------------------------------


