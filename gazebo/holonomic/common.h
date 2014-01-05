#ifndef _COMMON_H_
#define _COMMON_H_

//-----------------------------------------------------------------------------

#include <ostream>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>
#include <limits>

//-----------------------------------------------------------------------------

#include <boost/shared_ptr.hpp>

//-----------------------------------------------------------------------------

class ship_c;
typedef boost::shared_ptr<ship_c> ship_p;

class space_c;
//-----------------------------------------------------------------------------

typedef void (*ode_f)( const std::vector<double>& q, const std::vector<double>& u, std::vector<double>& dq, const ship_p ship );

typedef unsigned int (*best_control_f)( const ship_p pred, const ship_p prey, const std::vector<double>& q_in, std::vector<double>& q_out, const std::vector<double>& u0, std::vector<double>& u );

typedef void (*prey_command_f)( const std::vector<double>& pred_q, const std::vector<double>& prey_q, std::vector<double>& prey_u, ship_p pred, ship_p prey, const double& t );

//-----------------------------------------------------------------------------

#endif // _COMMON_H_
