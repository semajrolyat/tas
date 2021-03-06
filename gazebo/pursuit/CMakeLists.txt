cmake_minimum_required( VERSION 2.8 )

project( tas_gazebo_pursuit )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

add_library( car_controller SHARED car.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( car_controller ${GAZEBO_LIBRARIES} ${TAS_LIBRARIES} )

# Copy data files
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/car.world DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/pursuit.world DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )

# Copy setup file and substitute correct path into file
SET( SEARCH_REGEX "%BUILD_DIR%" )
file( READ ${CMAKE_CURRENT_SOURCE_DIR}/setup.sh SETUP_FILE_TEXT )
STRING( REGEX REPLACE "${SEARCH_REGEX}" "${CMAKE_CURRENT_BINARY_DIR}" MODIFIED_FILE_TEXT "${SETUP_FILE_TEXT}" )
FILE( WRITE ${CMAKE_CURRENT_BINARY_DIR}/setup.sh "${MODIFIED_FILE_TEXT}" )

