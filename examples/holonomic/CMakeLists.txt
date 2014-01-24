project( tas_ship )
cmake_minimum_required(VERSION 2.8)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g")

#find_package( RT REQUIRED )
#find_package( DL REQUIRED )
#find_package( Pthread REQUIRED )
#find_package( Moby REQUIRED )
#find_package( OMPL REQUIRED )
#find_package( GSL REQUIRED )
#find_package( Boost REQUIRED)

#include( FindPkgConfig )
#if( PKG_CONFIG_FOUND )
#  pkg_check_modules( GAZEBO gazebo )
#endif()
#include_directories( ${GAZEBO_INCLUDE_DIRS} )
#link_directories( ${GAZEBO_LIBRARY_DIRS} )

#include( FindPkgConfig )
#if( PKG_CONFIG_FOUND )
#  pkg_check_modules( RAVELIN ravelin )
#endif()
#include_directories( ${RAVELIN_INCLUDE_DIRS} )
#link_directories( ${RAVELIN_LIBRARY_DIRS} )


set( SHIP_HEADERS
  #${TAS_HEADERS}
  ${CMAKE_CURRENT_SOURCE_DIR}/cpu.h
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamics.h
  ${CMAKE_CURRENT_SOURCE_DIR}/error.h
  ${CMAKE_CURRENT_SOURCE_DIR}/experiment.h
  ${CMAKE_CURRENT_SOURCE_DIR}/ipc.h
  ${CMAKE_CURRENT_SOURCE_DIR}/log.h
  ${CMAKE_CURRENT_SOURCE_DIR}/os.h
  ${CMAKE_CURRENT_SOURCE_DIR}/schedule.h
  ${CMAKE_CURRENT_SOURCE_DIR}/tas.h
  ${CMAKE_CURRENT_SOURCE_DIR}/thread.h
  ${CMAKE_CURRENT_SOURCE_DIR}/time.h

  ${CMAKE_CURRENT_SOURCE_DIR}/common.h
  ${CMAKE_CURRENT_SOURCE_DIR}/constants.h
  ${CMAKE_CURRENT_SOURCE_DIR}/utilities.h
  ${CMAKE_CURRENT_SOURCE_DIR}/state.h
  ${CMAKE_CURRENT_SOURCE_DIR}/command.h
  ${CMAKE_CURRENT_SOURCE_DIR}/aabb.h
  ${CMAKE_CURRENT_SOURCE_DIR}/goal.h
  ${CMAKE_CURRENT_SOURCE_DIR}/control_space.h
  ${CMAKE_CURRENT_SOURCE_DIR}/integrator.h
  ${CMAKE_CURRENT_SOURCE_DIR}/state_propagator.h
  ${CMAKE_CURRENT_SOURCE_DIR}/control_sampler.h
  ${CMAKE_CURRENT_SOURCE_DIR}/plane.h 
  ${CMAKE_CURRENT_SOURCE_DIR}/space.h
  ${CMAKE_CURRENT_SOURCE_DIR}/maze.h
  ${CMAKE_CURRENT_SOURCE_DIR}/sensor.h
  ${CMAKE_CURRENT_SOURCE_DIR}/memory.h
  ${CMAKE_CURRENT_SOURCE_DIR}/message.h

  ${CMAKE_CURRENT_SOURCE_DIR}/ship.h

  ${CMAKE_CURRENT_SOURCE_DIR}/controller.h
  ${CMAKE_CURRENT_SOURCE_DIR}/planner.h
)

set( SHIP_SOURCES
  #${TAS_SOURCES} 
  ${CMAKE_CURRENT_SOURCE_DIR}/command.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/state.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/utilities.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/error.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/log.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cpu.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/time.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/os.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ship.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/space.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/thread.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/planner.cpp
)

set( SHIP_INCLUDE_DIRS
  ${MOBY_INCLUDE_DIR}
  ${BOOST_INCLUDE_DIR}
  #${CMAKE_CURRENT_SOURCE_DIR}
)

set( SHIP_LIBRARIES
  ${RT_LIBRARIES}
  ${DL_LIBRARIES}
  ${PTHREAD_LIBRARIES}
  ${MOBY_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${GSL_LIBRARIES}
  ${BOOST_LIBRARIES}
  ${RAVELIN_LIBRARIES}
  libboost_system.so
  libRavelin.so
  #${GAZEBO_LIBRARIES}
  #${TAS_LIBRARIES}
)

include_directories( ${SHIP_INCLUDE_DIRS} )

# Copy data files
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/pursuit.xml DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )

add_executable( tas-ship-controller ${CMAKE_CURRENT_SOURCE_DIR}/proc_controller.cpp ${SHIP_HEADERS} ${SHIP_SOURCES} )
target_link_libraries( tas-ship-controller ${SHIP_LIBRARIES} )

add_executable( tas-ship-planner ${CMAKE_CURRENT_SOURCE_DIR}/proc_planner.cpp ${SHIP_HEADERS} ${SHIP_SOURCES} )
target_link_libraries( tas-ship-planner ${SHIP_LIBRARIES} )

add_library( tas-ship-dynamics "MODULE" ${CMAKE_CURRENT_SOURCE_DIR}/plugin_dynamics.cpp ${SHIP_HEADERS} ${SHIP_SOURCES} )
target_link_libraries( tas-ship-dynamics ${SHIP_LIBRARIES} )

add_executable( tas-ship-coordinator ${CMAKE_CURRENT_SOURCE_DIR}/coordinator.cpp ${SHIP_HEADERS} ${SHIP_SOURCES} )
target_link_libraries( tas-ship-coordinator ${SHIP_LIBRARIES} )

