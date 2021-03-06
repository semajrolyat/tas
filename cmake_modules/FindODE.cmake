# find ODE (Open Dynamics Engine) includes and library
#
# ODE_INCLUDE_DIR - where the directory containing the ODE headers can be
#                   found
# ODE_LIBRARY     - full path to the ODE library
# ODE_FOUND       - TRUE if ODE was found

IF(NOT ODE_FOUND)

    FIND_PATH(ODE_INCLUDE_DIR ode/ode.h
        /usr/include
        /usr/local/include
        #$ENV{OGRE_HOME}/include # OGRE SDK on WIN32
        $ENV{INCLUDE}
    )
    FIND_LIBRARY(ODE_LIBRARY
        NAMES ode
        PATHS
        /usr/lib
        /usr/local/lib
        #$ENV{OGRE_HOME}/lib # OGRE SDK on WIN32
    )


    IF(ODE_INCLUDE_DIR)
        MESSAGE(STATUS "Found ODE include dir: ${ODE_INCLUDE_DIR}")
    ELSE(ODE_INCLUDE_DIR)
        MESSAGE(STATUS "Could NOT find ODE headers.")
    ENDIF(ODE_INCLUDE_DIR)

    IF(ODE_LIBRARY)
        MESSAGE(STATUS "Found ODE library: ${ODE_LIBRARY}")
    ELSE(ODE_LIBRARY)
        MESSAGE(STATUS "Could NOT find ODE library.")
    ENDIF(ODE_LIBRARY)

    IF(ODE_INCLUDE_DIR AND ODE_LIBRARY)
        SET(ODE_FOUND TRUE CACHE STRING "Whether ODE was found or not")
    ELSE(ODE_INCLUDE_DIR AND ODE_LIBRARY)
        SET(ODE_FOUND FALSE)
        IF(ODE_FIND_REQUIRED)
            MESSAGE(FATAL_ERROR "Could not find ODE. Please install ODE (http://www.ode.org)")
        ENDIF(ODE_FIND_REQUIRED)
    ENDIF(ODE_INCLUDE_DIR AND ODE_LIBRARY)

ENDIF(NOT ODE_FOUND)

#INCLUDE (FindPkgConfig)

#SET(REQUIRED_ODE_VERSION ${ODE_VERSION})

########################################
# Find packages
#IF (PKG_CONFIG_FOUND)
#  pkg_check_modules(ODE ode>=${ODE_VERSION})
#  pkg_check_modules(ODE)
#ENDIF (PKG_CONFIG_FOUND)

# if we didnt find it in pkg-config try ode-config
#IF (NOT ODE_FOUND)
#  SET(ODE_CONFIG_PATH "ode-config")
  #EXEC_PROGRAM(${ODE_CONFIG_PATH} ARGS --version RETURN_VALUE ODE_CONFIG_RETURN OUTPUT_VARIABLE DETECTED_ODE_VERSION )
  #STRING(REGEX REPLACE "[\r\n]" " " ${DETECTED_ODE_VERSION} "${${DETECTED_ODE_VERSION}}")

  # ode-config exists then get the details
#  IF(NOT ODE_CONFIG_RETURN)
#    IF(NOT ${REQUIRED_ODE_VERSION} VERSION_LESS ${DETECTED_ODE_VERSION} )
#      MESSAGE(STATUS "  ode-config reports version ${DETECTED_ODE_VERSION}")
#      EXEC_PROGRAM(${ODE_CONFIG_PATH} ARGS --cflags RETURN_VALUE ODE_CONFIG_RETURN OUTPUT_VARIABLE ODE_INCLUDE_DIRS )
#      STRING(REGEX REPLACE "[\r\n]" " " ${ODE_INCLUDE_DIRS} "${${ODE_INCLUDE_DIRS}}")

      # we want to extract -DdSINGLE or -DdDOUBLE to compile with the correct flag
#      STRING(REGEX REPLACE ".*-Dd([^ ]+).*" "-Dd\\1" ODE_FLAGS_OTHER "${ODE_INCLUDE_DIRS}")

#      EXEC_PROGRAM(${ODE_CONFIG_PATH} ARGS --libs RETURN_VALUE ODE_CONFIG_RETURN OUTPUT_VARIABLE ODE_LDFLAGS )
#      STRING(REGEX REPLACE "[\r\n]" " " ${ODE_LDFLAGS} "${${ODE_LDFLAGS}}")

#      SET(ODE_FOUND ${DETECTED_ODE_VERSION})
#    ELSE(NOT ${REQUIRED_ODE_VERSION} VERSION_LESS ${DETECTED_ODE_VERSION} )
#      MESSAGE(STATUS "ode-config reports wrong version (${DETECTED_ODE_VERSION} < ${REQUIRED_ODE_VERSION})")
#    ENDIF(NOT ${REQUIRED_ODE_VERSION} VERSION_LESS ${DETECTED_ODE_VERSION} )
#  ELSE(NOT ODE_CONFIG_RETURN)
#    MESSAGE(STATUS "no ode-config found")
#  ENDIF(NOT ODE_CONFIG_RETURN)
#ENDIF (NOT ODE_FOUND)

#IF (NOT ODE_FOUND)
#  BUILD_ERROR ("ODE and development files not found. See the following website: http://www.ode.org")
#ELSE (NOT ODE_FOUND)
  #SET (CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${ODE_CFLAGS_OTHER}" CACHE INTERNAL "added dSINGLE" FORCE)
  #SET (CMAKE_C_FLAGS_DEBUG   "${CMAKE_C_FLAGS_DEBUG}   ${ODE_CFLAGS_OTHER}" CACHE INTERNAL "added dSINGLE" FORCE)
  #SET (CMAKE_C_FLAGS_PROFILE "${CMAKE_C_FLAGS_PROFILE} ${ODE_CFLAGS_OTHER}" CACHE INTERNAL "added dSINGLE" FORCE)
  #MESSAGE (STATUS "\n\ndebug\n" ${CMAKE_C_FLAGS_DEBUG} "\n\n")

#  APPEND_TO_CACHED_LIST(gazeboserver_cflags
#                        ${gazeboserver_cflags_desc}
#                        ${ODE_CFLAGS_OTHER})

#  APPEND_TO_CACHED_LIST(gazeboserver_include_dirs 
#                        ${gazeboserver_include_dirs_desc} 
#                        ${ODE_INCLUDE_DIRS})
#  APPEND_TO_CACHED_LIST(gazeboserver_link_dirs 
#                        ${gazeboserver_link_dirs_desc} 
#                        ${ODE_LIBRARY_DIRS})
#  APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
#                        ${gazeboserver_link_libs_desc} 
#                        ${ODE_LINK_LIBS})
#  APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
#                        ${gazeboserver_link_libs_desc} 
#                        ${ODE_LIBRARIES})
#  APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
#                        ${gazeboserver_link_libs_desc} 
#                        ${ODE_LDFLAGS})
#ENDIF (NOT ODE_FOUND)
