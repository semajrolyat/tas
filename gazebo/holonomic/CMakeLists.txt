cmake_minimum_required( VERSION 2.8 )

project( tas_gazebo_holonomic_ship )

#add_library( space_controller SHARED space.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
#target_link_libraries( space_controller ${GAZEBO_LIBRARIES} ${TAS_LIBRARIES} )

set( SHIP_HEADERS
    ${TAS_HEADERS}
    ${CMAKE_CURRENT_SOURCE_DIR}/constants.h
    ${CMAKE_CURRENT_SOURCE_DIR}/utilities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/state.h
    ${CMAKE_CURRENT_SOURCE_DIR}/command.h
    ${CMAKE_CURRENT_SOURCE_DIR}/aabb.h
    ${CMAKE_CURRENT_SOURCE_DIR}/goal.h
    ${CMAKE_CURRENT_SOURCE_DIR}/control_space.h
    ${CMAKE_CURRENT_SOURCE_DIR}/integrator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/state_propagator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/directed_control_sampler.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plane.h 
    ${CMAKE_CURRENT_SOURCE_DIR}/space.h
    ${CMAKE_CURRENT_SOURCE_DIR}/maze.h
)

set( SHIP_SOURCES
    ${TAS_SOURCES} 
)

set( SHIP_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set( SHIP_LIBRARIES
    ${GAZEBO_LIBRARIES}
    ${RAVELIN_LIBRARIES}
    ${TAS_LIBRARIES}
    libboost_system.so
    libRavelin.so
)

include_directories( ${SHIP_INCLUDE_DIRS} )

#add_executable( maze space.cpp ${SHIP_HEADERS} ${SHIP_SOURCES} )
#target_link_libraries( maze ${SHIP_LIBRARIES} )

add_library( ship_controller SHARED ship.cpp space.cpp ${SHIP_HEADERS} ${SHIP_SOURCES} )
target_link_libraries( ship_controller ${SHIP_LIBRARIES} )

# Copy data files
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/ship.world DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/pursuit.world DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )

# Copy setup file and substitute correct path into file
SET( SEARCH_REGEX "%BUILD_DIR%" )
file( READ ${CMAKE_CURRENT_SOURCE_DIR}/setup.sh SETUP_FILE_TEXT )
STRING( REGEX REPLACE "${SEARCH_REGEX}" "${CMAKE_CURRENT_BINARY_DIR}" MODIFIED_FILE_TEXT "${SETUP_FILE_TEXT}" )
FILE( WRITE ${CMAKE_CURRENT_BINARY_DIR}/setup.sh "${MODIFIED_FILE_TEXT}" )

