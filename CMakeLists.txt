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
find_package (LibXml2 REQUIRED)

message( ${MOBY_LIBRARIES} )
message( ${MOBY_INCLUDE_DIR} )

if (LIBXML2_FOUND)
  set (CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${LIBXML2_DEFINITIONS})
  set (CMAKE_CXX_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG} ${LIBXML2_DEFINITIONS})
  set (CMAKE_CXX_FLAGS_RELWITHDEBINFO ${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${LIBXML2_DEFINITIONS})
  set (CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE} ${LIBXML2_DEFINITIONS})
  set (CMAKE_CXX_FLAGS_MINSIZEREL ${CMAKE_CXX_FLAGS_MINSIZEREL} ${LIBXML2_DEFINITIONS})
  include_directories (${LIBXML2_INCLUDE_DIR})
endif (LIBXML2_FOUND)

include_directories( ${MOBY_INCLUDE_DIR} )

set( TAS_LIBRARIES
    ${RT_LIBRARIES}
    ${DL_LIBRARIES}
    ${PTHREAD_LIBRARIES}
)

add_executable( proto_sim proto_sim.cpp )
target_link_libraries( proto_sim ${TAS_LIBRARIES} )

add_executable( controller controller.cpp )
target_link_libraries( controller ${TAS_LIBRARIES} )

add_executable( ut_controllerexec ut_controllerexec.cpp )
target_link_libraries( ut_controllerexec ${TAS_LIBRARIES} )

add_executable( ut_controllerticking ut_controllerticking.cpp )
target_link_libraries( ut_controllerticking ${TAS_LIBRARIES} )

add_executable( ut_controllerschedule ut_controllerschedule.cpp )
target_link_libraries( ut_controllerschedule ${TAS_LIBRARIES} )

add_executable( ut_controllertiming ut_controllertiming.cpp )
target_link_libraries( ut_controllertiming ${TAS_LIBRARIES} )

add_executable( proto_threading proto_threading.cpp )
target_link_libraries( proto_threading ${TAS_LIBRARIES} )

add_library( plugin "MODULE" plugin.cpp )
target_link_libraries( plugin ${TAS_LIBRARIES} ${MOBY_LIBRARIES} )
