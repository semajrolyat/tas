# Install script for directory: /home/james/tas/ros/src/pendulum/gazebo

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/james/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}/home/james/local/lib/gazebo_plugins/libros_pendulum_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/james/local/lib/gazebo_plugins/libros_pendulum_plugin.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/james/local/lib/gazebo_plugins/libros_pendulum_plugin.so"
         RPATH "/opt/ros/groovy/lib")
  ENDIF()
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/james/local/lib/gazebo_plugins/libros_pendulum_plugin.so")
FILE(INSTALL DESTINATION "/home/james/local/lib/gazebo_plugins" TYPE SHARED_LIBRARY FILES "/home/james/tas/ros/src/pendulum/gazebo/build/libros_pendulum_plugin.so")
  IF(EXISTS "$ENV{DESTDIR}/home/james/local/lib/gazebo_plugins/libros_pendulum_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/james/local/lib/gazebo_plugins/libros_pendulum_plugin.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/james/local/lib/gazebo_plugins/libros_pendulum_plugin.so"
         OLD_RPATH "/usr/lib/gazebo-1.8/plugins:/opt/ros/groovy/lib:/home/james/tas/ros/install/lib:"
         NEW_RPATH "/opt/ros/groovy/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/james/local/lib/gazebo_plugins/libros_pendulum_plugin.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}/home/james/local/lib/gazebo_plugins/libros_model_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/james/local/lib/gazebo_plugins/libros_model_plugin.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/james/local/lib/gazebo_plugins/libros_model_plugin.so"
         RPATH "/opt/ros/groovy/lib")
  ENDIF()
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/james/local/lib/gazebo_plugins/libros_model_plugin.so")
FILE(INSTALL DESTINATION "/home/james/local/lib/gazebo_plugins" TYPE SHARED_LIBRARY FILES "/home/james/tas/ros/src/pendulum/gazebo/build/libros_model_plugin.so")
  IF(EXISTS "$ENV{DESTDIR}/home/james/local/lib/gazebo_plugins/libros_model_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/james/local/lib/gazebo_plugins/libros_model_plugin.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/james/local/lib/gazebo_plugins/libros_model_plugin.so"
         OLD_RPATH "/usr/lib/gazebo-1.8/plugins:/opt/ros/groovy/lib:/home/james/tas/ros/install/lib:"
         NEW_RPATH "/opt/ros/groovy/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/james/local/lib/gazebo_plugins/libros_model_plugin.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/james/tas/ros/src/pendulum/gazebo/build/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/james/tas/ros/src/pendulum/gazebo/build/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
