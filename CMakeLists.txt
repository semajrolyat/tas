# For Debug Build :: -DCMAKE_BUILD_TYPE=Debug

project( TAS )
cmake_minimum_required(VERSION 2.8)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g")

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

find_package( RT REQUIRED )
find_package( DL REQUIRED )
find_package( Pthread REQUIRED )
find_package( Moby REQUIRED )

#message( ${MOBY_LIBRARIES} )
#message( ${MOBY_INCLUDE_DIR} )

set( TAS_HEADERS
    tas.h
    actuator.h
    dynamics_plugin.h
)

set( TAS_SOURCES )

set( TAS_INCLUDE_DIRS
    ${MOBY_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set( TAS_LIBRARIES
    ${RT_LIBRARIES}
    ${DL_LIBRARIES}
    ${PTHREAD_LIBRARIES}
    ${MOBY_LIBRARIES}
)

include_directories( ${TAS_INCLUDE_DIRS} )

add_executable( controller ${CMAKE_CURRENT_SOURCE_DIR}/controller.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( controller ${TAS_LIBRARIES} )

add_executable( coordinator ${CMAKE_CURRENT_SOURCE_DIR}/coordinator.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( coordinator ${TAS_LIBRARIES} )

add_library( dynamics "MODULE" ${CMAKE_CURRENT_SOURCE_DIR}/dynamics.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( dynamics ${TAS_LIBRARIES} )
