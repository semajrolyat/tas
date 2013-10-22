cmake_minimum_required(VERSION 2.8.3)
project(pendulum)

#find_package(gazebo_ros)

#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation gazebo_ros)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

#find_package(Boost REQUIRED COMPONENTS system)

include( FindPkgConfig )
if( PKG_CONFIG_FOUND )
  pkg_check_modules( GAZEBO gazebo )
endif()

#######################################
## Declare ROS messages and services ##
#######################################

#add_message_files(DIRECTORY msg FILES command.msg synch.msg)
#add_service_files(DIRECTORY srv FILES state.srv)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  synch.msg
  command.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   rpc_command.srv
   state.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES pendulum
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
   DEPENDS system_lib gazebo_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(pendulum
#   src/${PROJECT_NAME}/pendulum.cpp
# )

## Declare a cpp executabli
#add_executable(talker src/talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker pendulum_generate_message_cpp )

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener pendulum_generate_message_cpp )

#add_executable(command_talker src/command_talker.cpp)
#target_link_libraries(command_talker ${catkin_LIBRARIES})
#add_dependencies(command_talker pendulum_generate_message_cpp )

#add_executable(command_listener src/command_listener.cpp)
#target_link_libraries(command_listener ${catkin_LIBRARIES})
#add_dependencies(command_listener pendulum_generate_message_cpp )

#add_executable(ros_pendulum_standup_controller src/ros_pendulum_standup_controller.cpp)
#target_link_libraries(ros_pendulum_standup_controller ${catkin_LIBRARIES})
#add_dependencies(ros_pendulum_standup_controller pendulum_generate_message_cpp )

add_executable(rpc_standup_controller src/rpc_standup_controller.cpp)
target_link_libraries(rpc_standup_controller ${catkin_LIBRARIES})
add_dependencies(rpc_standup_controller pendulum_generate_message_cpp )

add_library(rpc_gazebo_robot SHARED src/rpc_gazebo_robot.cpp)
target_link_libraries(rpc_gazebo_robot ${catkin_LIBRARIES})
add_dependencies(rpc_gazebo_robot pendulum_generate_message_cpp )


add_executable(standup_controller src/standup_controller.cpp)
target_link_libraries(standup_controller ${catkin_LIBRARIES})
add_dependencies(standup_controller pendulum_generate_message_cpp )

add_library(gazebo_robot SHARED src/gazebo_robot.cpp)
target_link_libraries(gazebo_robot ${catkin_LIBRARIES})
add_dependencies(gazebo_robot pendulum_generate_message_cpp )



## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pendulum_node pendulum_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(pendulum_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS standup_controller gazebo_robot 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pendulum.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
