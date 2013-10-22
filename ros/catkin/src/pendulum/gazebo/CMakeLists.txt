cmake_minimum_required(VERSION 2.8)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo REQUIRED)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include(cmake/rospack.cmake)

get_rospack_flags(roscpp)
#get_rospack_flags(pendulum)
message(${pendulum_INCLUDE_DIRS})

include_directories(${roscpp_INCLUDE_DIRS} ${pendulum_INCLUDE_DIRS})
link_directories(${roscpp_LIBRARY_DIRS} ${pendulum_LIBRARY_DIRS})
set(CMAKE_INSTALL_RPATH "${roscpp_LIBRARY_DIRS}")
add_library(ros_pendulum_plugin SHARED ros_pendulum_plugin.cc)
set_target_properties(ros_pendulum_plugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(ros_pendulum_plugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(ros_pendulum_plugin ${roscpp_LIBRARIES} ${pendulum_LIBRARIES})
install (TARGETS ros_pendulum_plugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)

include_directories(${roscpp_INCLUDE_DIRS})
link_directories(${roscpp_LIBRARY_DIRS})
set(CMAKE_INSTALL_RPATH "${roscpp_LIBRARY_DIRS}")
add_library(ros_model_plugin SHARED ros_model_plugin.cc)
set_target_properties(ros_model_plugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(ros_model_plugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(ros_model_plugin ${roscpp_LIBRARIES})
install (TARGETS ros_model_plugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)

