#include <CubicSpline.h>

// Note: Everything is static for this class so don't need ctor/dtor
//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
/// Default Constructor
CubicSpline::CubicSpline( void ) {

}
//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
/// Destructor
CubicSpline::~CubicSpline( void ) {

}
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
/// Returns the Catmull-Rom basis matrix M
Moby::MatrixN CubicSpline::basisCatmullRom( void ) {
    double a = 0.5;
    Moby::MatrixN M(4,4);

    M(0,0) = -1.0;      M(0,1) =  3.0;      M(0,2) = -3.0;      M(0,3) =  1.0;
    M(1,0) =  2.0;      M(1,1) = -5.0;      M(1,2) =  4.0;      M(1,3) = -1.0;
    M(2,0) = -1.0;      M(2,1) =  0.0;      M(2,2) =  1.0;      M(2,3) =  0.0;
    M(3,0) =  0.0;      M(3,1) =  2.0;      M(3,2) =  0.0;      M(3,3) =  0.0;

    return M * a;
}
//------------------------------------------------------------------------------
/// Returns the Uniform Non-Rational B-Spline basis matrix M
Moby::MatrixN CubicSpline::basisUniformNonrationalBSpline( void ) {
    double a = 1.0/6.0;
    Moby::MatrixN M(4,4);

    M(0,0) = -1.0;      M(0,1) =  3.0;      M(0,2) = -3.0;      M(0,3) =  1.0;
    M(1,0) =  3.0;      M(1,1) = -6.0;      M(1,2) =  3.0;      M(1,3) =  0.0;
    M(2,0) = -3.0;      M(2,1) =  0.0;      M(2,2) =  3.0;      M(2,3) =  0.0;
    M(3,0) =  1.0;      M(3,1) =  4.0;      M(3,2) =  1.0;      M(3,3) =  0.0;

    return M * a;
}
//------------------------------------------------------------------------------
/// Returns the Bezier basis matrix M
Moby::MatrixN CubicSpline::basisBezier( void ) {
    Moby::MatrixN M(4,4);

    M(0,0) = -1.0;      M(0,1) =  3.0;      M(0,2) = -3.0;      M(0,3) =  1.0;
    M(1,0) =  3.0;      M(1,1) = -6.0;      M(1,2) =  3.0;      M(1,3) =  0.0;
    M(2,0) = -3.0;      M(2,1) =  3.0;      M(2,2) =  0.0;      M(2,3) =  0.0;
    M(3,0) =  1.0;      M(3,1) =  0.0;      M(3,2) =  0.0;      M(3,3) =  0.0;

    return M;
}
//------------------------------------------------------------------------------
/// Returns the Hermite basis matrix M
Moby::MatrixN CubicSpline::basisHermite( void ) {
    Moby::MatrixN M(4,4);

    M(0,0) =  2.0;      M(0,1) = -2.0;      M(0,2) =  1.0;      M(0,3) =  1.0;
    M(1,0) = -3.0;      M(1,1) =  3.0;      M(1,2) = -2.0;      M(1,3) = -1.0;
    M(2,0) =  0.0;      M(2,1) =  0.0;      M(2,2) =  1.0;      M(2,3) =  0.0;
    M(3,0) =  1.0;      M(3,1) =  0.0;      M(3,2) =  0.0;      M(3,3) =  0.0;

    return M;
}

//------------------------------------------------------------------------------
/// Returns the 3-space position vector at point u along spline Q
/// given blended matrix C and point u
/// where u E [0,1]
Moby::Vector3 CubicSpline::position( const Moby::MatrixN& blend_matrix, const double& u ) {
    double u2 = u * u;
    double u3 = u2 * u;

    Moby::MatrixN U(1,4);
    U(0,0) = u3;
    U(0,1) = u2;
    U(0,2) = u;
    U(0,3) = 1;

    Moby::MatrixN Q = U * blend_matrix;

    return Moby::Vector3( Q(0,0), Q(0,1), Q(0,2) );
}

//------------------------------------------------------------------------------
/// Returns the blended matrix C
/// given the basis matrix M, the ordered set of control points defining spline Q
/// and the index of the segment of the desired sub spline.
/// where C is the concatenation of basis matrix M and geometry matrix G, e.g. C = M * G,
/// and segment is an integer on the interval [0,n-4] | n = number of control points
Moby::MatrixN CubicSpline::blend( const Moby::MatrixN& basis_matrix, const std::vector<Moby::Vector3>& ctlpts, const unsigned int& segment ) {
    assert( is_segment_valid( ctlpts, segment ) );

    Moby::MatrixN geometry_matrix = geometry( ctlpts, segment );

    return basis_matrix * geometry_matrix;
}

//------------------------------------------------------------------------------
/// Returns the 4 x 3 geometry matrix (G)
/// given the ordered set of control points and the index of the segment
/// the desired sub spline.
/// where segment is an integer on the interval [0,n-4] | n = number of control points
Moby::MatrixN CubicSpline::geometry( const std::vector<Moby::Vector3>& ctlpts, const unsigned int& segment ) {
    assert( is_segment_valid( ctlpts, segment ) );

    std::vector<Moby::Vector3>::const_iterator it0 = ctlpts.begin() + segment;
    std::vector<Moby::Vector3>::const_iterator it1 = it0+1;
    std::vector<Moby::Vector3>::const_iterator it2 = it0+2;
    std::vector<Moby::Vector3>::const_iterator it3 = it0+3;

    Moby::MatrixN G(4,3);
    G(0,0) = (*it0)[0];      G(0,1) = (*it0)[1];      G(0,2) = (*it0)[2];
    G(1,0) = (*it1)[0];      G(1,1) = (*it1)[1];      G(1,2) = (*it1)[2];
    G(2,0) = (*it2)[0];      G(2,1) = (*it2)[1];      G(2,2) = (*it2)[2];
    G(3,0) = (*it3)[0];      G(3,1) = (*it3)[1];      G(3,2) = (*it3)[2];

    return G;
}

//------------------------------------------------------------------------------

unsigned int CubicSpline::control_points( const std::vector<Moby::Vector3>& ctlpts ) {
    return ctlpts.size();
}

//------------------------------------------------------------------------------

unsigned int CubicSpline::segments( const std::vector<Moby::Vector3>& ctlpts ) {
    return ctlpts.size() - 3;
}

//------------------------------------------------------------------------------

bool CubicSpline::is_cubic_spline( const std::vector<Moby::Vector3>& ctlpts ) {
    if( ctlpts.size() < 3 )
        return false;
    return true;
}

//------------------------------------------------------------------------------

bool CubicSpline::is_control_point_valid( const std::vector<Moby::Vector3>& ctlpts, const unsigned int& qid ) {
    if( qid >= ctlpts.size() )
        return false;
    return true;
}

//------------------------------------------------------------------------------

bool CubicSpline::is_segment_valid( const std::vector<Moby::Vector3>& ctlpts, const unsigned int& segment ) {
    if( segment >= segments( ctlpts ) )
        return false;
    return true;
}

//------------------------------------------------------------------------------

std::vector<Moby::Vector3> CubicSpline::segment( const std::vector<Moby::Vector3>& ctlpts, const unsigned int& idx ) {
    assert( is_segment_valid( ctlpts, idx ) );

    std::vector<Moby::Vector3>::const_iterator it0 = ctlpts.begin() + idx;
    std::vector<Moby::Vector3>::const_iterator it1 = it0 + 1;
    std::vector<Moby::Vector3>::const_iterator it2 = it0 + 2;
    std::vector<Moby::Vector3>::const_iterator it3 = it0 + 3;

    std::vector<Moby::Vector3> result;
    result.push_back( *it0 );
    result.push_back( *it1 );
    result.push_back( *it2 );
    result.push_back( *it3 );

    return result;
}

//------------------------------------------------------------------------------

std::vector< std::pair< double, double > > CubicSpline::map_arclengths( const Moby::MatrixN& basis_matrix, const std::vector<Moby::Vector3>& ctlpts ) {
    std::vector< std::pair< double, double > > result;

    double s = 0.0;
    unsigned int n = segments( ctlpts );
    for( unsigned int i = 0; i < n; i++ ) {
        Moby::MatrixN blend_matrix = blend( basis_matrix, ctlpts, i );
        double ds = linearly_interpolate_arclength( blend_matrix, 0.0, 1.0, 0.05 );
        s += ds;
        result.push_back( std::pair< double, double >( ds, s ) );
    }

    return result;
}


//------------------------------------------------------------------------------
double CubicSpline::linearly_approximate_arclength( const Moby::MatrixN& blend_matrix, const double& u0, const double& u1 ) {
    assert( u0 >= 0.0 && u0 < u1 && u1 <= 1.0 );

    Moby::Vector3 v0 = position( blend_matrix, u0 );
    Moby::Vector3 v1 = position( blend_matrix, u1 );

    return line_segment_length( v0, v1 );
}

//------------------------------------------------------------------------------
double CubicSpline::linearly_interpolate_arclength( const Moby::MatrixN& blend_matrix, const double& u0, const double& u1, const double& du ) {
    assert( u0 >= 0.0 && u0 < u1 && u1 <= 1.0 );
    assert( du < u1 - u0 );

    std::vector<Moby::Vector3> points;
    double s = 0.0;
    Moby::Vector3 vi, vi1;

    // query the points and store for processing
    for( double u = u0; u < u1; u += du ) {
        points.push_back( position( blend_matrix, u ) );
    }
    // step size may have not been a factor of u1-u0 so pick up odd case
    points.push_back( position( blend_matrix, u1 ) );

    // process each pair of table entries and append length
    for( std::vector<Moby::Vector3>::iterator it = points.begin(); it != points.end(); it++ ) {
        if( it == points.begin() ) {
            vi = *it;
        } else {
            vi1 = *it;
            s += line_segment_length( vi1, vi );
            vi = *it;
        }
    }

    // return the interpolated arclength s
    return s;
}

//------------------------------------------------------------------------------
double CubicSpline::linearly_interpolate_arclength( const Moby::MatrixN& basis_matrix, const std::vector<Moby::Vector3>& ctlpts ) {
    double s = 0.0;

    unsigned int n = segments( ctlpts );
    for( unsigned int i = 0; i < n; i++ ) {
        Moby::MatrixN blend_matrix = blend( basis_matrix, ctlpts, i );
        s += linearly_interpolate_arclength( blend_matrix, 0.0, 1.0, 0.05 );
    }
    return s;
}

//------------------------------------------------------------------------------

double CubicSpline::line_segment_length( const Moby::Vector3& p1, const Moby::Vector3& p2 ) {
    double dx = 0.0, dy = 0.0, dz = 0.0;

    dx = fabs( p2[0] - p1[0] );
    dy = fabs( p2[1] - p1[1] );
    dz = fabs( p2[2] - p1[2] );

    return sqrt( dx*dx + dy*dy + dz*dz );
}
