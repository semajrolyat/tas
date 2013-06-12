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

#set( TAS_HEADERS ) #individual files are set in the CMakeLists.txt in ./include
#set( TAS_SOURCES ) #individual files are set in the CMakeLists.txt in ./src
#add_subdirectory( "include" )
#add_subdirectory( "src" )

set( TAS_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/TAS.h
    ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorMessage.h
    ${CMAKE_CURRENT_SOURCE_DIR}/DynamicsPlugin.h
)

set( TAS_SOURCES )

set( TAS_INCLUDE_DIRS
    #${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MOBY_INCLUDE_DIR}
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

#add_executable( proto_sim proto_sim.cpp )
#target_link_libraries( proto_sim ${TAS_LIBRARIES} )

#add_executable( ut_controllerexec ut_controllerexec.cpp )
#target_link_libraries( ut_controllerexec ${TAS_LIBRARIES} )

#add_executable( ut_controllerticking ut_controllerticking.cpp )
#target_link_libraries( ut_controllerticking ${TAS_LIBRARIES} )

#add_executable( ut_controllerschedule ut_controllerschedule.cpp )
#target_link_libraries( ut_controllerschedule ${TAS_LIBRARIES} )

#add_executable( ut_controllertiming ut_controllertiming.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
#target_link_libraries( ut_controllertiming ${TAS_LIBRARIES} )

#add_executable( proto_threading proto_threading.cpp )
#target_link_libraries( proto_threading ${TAS_LIBRARIES} )

#add_library( plugin "MODULE" plugin.cpp )
#target_link_libraries( plugin ${TAS_LIBRARIES} )

## Plugins ##
#add_library( moby_driver "MODULE" moby_driver.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
#target_link_libraries( moby_driver ${TAS_LIBRARIES} )

## Moby Controllers ##
#add_library( inverted-pendulum-pd-controller "MODULE" inverted-pendulum-pd-controller.cpp )
#target_link_libraries(inverted-pendulum-pd-controller ${MOBY_LIBRARIES} )

## Utilities ##
#add_executable( tickspersec tickspersec.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
#target_link_libraries( tickspersec ${TAS_LIBRARIES} )

#---------------------

#add_subdirectory( "unittest" )

#add_subdirectory( "timing" )

#add_subdirectory( "commandbuffer" )

#add_subdirectory( "mutex" )

