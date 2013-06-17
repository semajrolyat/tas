#include <Trajectory.h>

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------

Trajectory::Trajectory( void ) {

}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------

Trajectory::~Trajectory( void ) {

}

//------------------------------------------------------------------------------
// Data Management
//------------------------------------------------------------------------------

void Trajectory::append_controlpoint( const Moby::Vector3& pt ) {
    controlpoints.push_back( pt );
}

void Trajectory::append_keyframe( const unsigned int& frame, const Moby::Vector3& pt ) {
    keyframes.push_back( std::pair< unsigned int, Moby::Vector3 >( frame, pt ) );
}


//------------------------------------------------------------------------------

