#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_

//------------------------------------------------------------------------------

#include <vector>
#include <utility>

#include <CubicSpline.h>

//------------------------------------------------------------------------------

class Trajectory {
public:
    Trajectory( void );
    virtual ~Trajectory( void );

    //void append( const Keyframe& keyframe );

    std::vector< std::pair< unsigned int, Moby::Vector3 > >     keyframes;
    std::vector< Moby::Vector3 >                                controlpoints;

    // ds for the current segment and s over all segments preceding this segment
    // including the length of this segment
    std::vector< std::pair< double, double > >                  arclengths;

    Moby::MatrixN basis_matrix;
    //Moby::MatrixN blend_matrix;

    void append_controlpoint( const Moby::Vector3& pt );
    void append_keyframe( const unsigned int& frame, const Moby::Vector3& pt );

};

//------------------------------------------------------------------------------

#endif // _TRAJECTORY_H_
