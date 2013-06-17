#ifndef _CUBICSPLINE_H_
#define _CUBICSPLINE_H_

//------------------------------------------------------------------------------

#include <Moby/Vector3.h>
#include <Moby/Matrix4.h>
#include <Moby/MatrixN.h>
#include <vector>

//------------------------------------------------------------------------------
/*
typedef enum {
    CUBIC_SPLINE_BEZIER,
    CUBIC_SPLINE_B,
    CUBIC_SPLINE_CATMULLROM,
    CUBIC_SPLINE_HERMITE
} ECubicSplineBasis;
*/
//------------------------------------------------------------------------------


class CubicSpline {
public:
    CubicSpline( void );
    virtual ~CubicSpline( void );

    static Moby::MatrixN basisCatmullRom( void );
    static Moby::MatrixN basisUniformNonrationalBSpline( void );
    static Moby::MatrixN basisBezier( void );
    static Moby::MatrixN basisHermite( void );

    static Moby::Vector3 position( const Moby::MatrixN& blend_matrix, const double& u );

    static Moby::MatrixN blend( const Moby::MatrixN& basis_matrix, const std::vector<Moby::Vector3> &ctlpts, const unsigned int& segment );
    static Moby::MatrixN geometry( const std::vector<Moby::Vector3>& ctlpts, const unsigned int& segment );

    static unsigned int control_points( const std::vector<Moby::Vector3>& ctlpts );
    static unsigned int segments( const std::vector<Moby::Vector3>& ctlpts );

    static bool is_cubic_spline( const std::vector<Moby::Vector3>& ctlpts );
    static bool is_control_point_valid( const std::vector<Moby::Vector3>& ctlpts, const unsigned int& qid  );
    static bool is_segment_valid( const std::vector<Moby::Vector3>& ctlpts, const unsigned int& segment );

    static std::vector<Moby::Vector3> segment( const std::vector<Moby::Vector3>& ctlpts, const unsigned int& idx );

    static std::vector< std::pair<double, double> > map_arclengths( const Moby::MatrixN& basis_matrix, const std::vector<Moby::Vector3>& ctlpts );

    static double linearly_approximate_arclength( const Moby::MatrixN& blend_matrix, const double& u0 = 0.0, const double& u1 = 1.0 );
    static double linearly_interpolate_arclength( const Moby::MatrixN& blend_matrix, const double& u0 = 0.0, const double& u1 = 1.0, const double& du = 0.1 );
    static double linearly_interpolate_arclength( const Moby::MatrixN& basis_matrix, const std::vector<Moby::Vector3>& ctlpts );

    static double line_segment_length( const Moby::Vector3& p1, const Moby::Vector3& p2 );
};

//------------------------------------------------------------------------------

#endif // _CUBICSPLINE_H_
