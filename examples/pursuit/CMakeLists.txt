project( pursuit )
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


set( PURSUIT_HEADERS
  #${TAS_HEADERS}
  ${CMAKE_CURRENT_SOURCE_DIR}/tcs.h
  ${CMAKE_CURRENT_SOURCE_DIR}/types.h
  ${CMAKE_CURRENT_SOURCE_DIR}/os.h
  ${CMAKE_CURRENT_SOURCE_DIR}/log.h
  ${CMAKE_CURRENT_SOURCE_DIR}/cpu.h
  ${CMAKE_CURRENT_SOURCE_DIR}/time.h
  ${CMAKE_CURRENT_SOURCE_DIR}/timer.h
  ${CMAKE_CURRENT_SOURCE_DIR}/notification.h
  ${CMAKE_CURRENT_SOURCE_DIR}/message.h
  ${CMAKE_CURRENT_SOURCE_DIR}/thread.h
  ${CMAKE_CURRENT_SOURCE_DIR}/thread_heap.h
  ${CMAKE_CURRENT_SOURCE_DIR}/scheduler.h
  ${CMAKE_CURRENT_SOURCE_DIR}/timesink.h
  ${CMAKE_CURRENT_SOURCE_DIR}/processor.h
  ${CMAKE_CURRENT_SOURCE_DIR}/osthread.h
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamics.h
  ${CMAKE_CURRENT_SOURCE_DIR}/channels.h  # needs generalization for API
)

set( PURSUIT_SOURCES
  #${TAS_SOURCES} 
  ${CMAKE_CURRENT_SOURCE_DIR}/os.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/log.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cpu.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/time.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/timer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/message.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/thread_heap.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/scheduler.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/timesink.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/processor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/osthread.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamics.cpp
)

set( PURSUIT_INCLUDE_DIRS
  ${MOBY_INCLUDE_DIR}
  ${BOOST_INCLUDE_DIR}
  #${CMAKE_CURRENT_SOURCE_DIR}
)

set( PURSUIT_LIBRARIES
  ${RT_LIBRARIES}
  ${DL_LIBRARIES}
  ${PTHREAD_LIBRARIES}
  ${MOBY_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${GSL_LIBRARIES}
  ${BOOST_LIBRARIES}
  ${RAVELIN_LIBRARIES}
  libboost_system-mt.so
  libRavelin.so
  #${GAZEBO_LIBRARIES}
  #${TAS_LIBRARIES}
)

include_directories( ${PURSUIT_INCLUDE_DIRS} )

# Copy data files
#file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/pursuit.xml DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )

#add_executable( pursuit-coordinator ${CMAKE_CURRENT_SOURCE_DIR}/coordinator.cpp ${PURSUIT_HEADERS} ${PURSUIT_SOURCES} )
#target_link_libraries( pursuit-coordinator ${PURSUIT_LIBRARIES} )

add_subdirectory( test )
