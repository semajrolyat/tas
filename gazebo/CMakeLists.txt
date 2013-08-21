cmake_minimum_required(VERSION 2.8)

project( tas_gazebo )

include( FindPkgConfig )
if( PKG_CONFIG_FOUND )
  pkg_check_modules( GAZEBO gazebo )
endif()
include_directories( ${GAZEBO_INCLUDE_DIRS} )
link_directories( ${GAZEBO_LIBRARY_DIRS} )

add_subdirectory( pendulum )

