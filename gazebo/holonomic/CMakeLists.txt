cmake_minimum_required( VERSION 2.8 )

project( tas_gazebo_holonomic_ship )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

add_library( space_controller SHARED space.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( space_controller ${GAZEBO_LIBRARIES} ${TAS_LIBRARIES} )

add_library( ship_controller SHARED ship.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( ship_controller ${GAZEBO_LIBRARIES} ${RAVELIN_LIBRARIES} ${TAS_LIBRARIES} libRavelin.so )

# Copy data files
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/ship.world DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/ship.world DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )

# Copy setup file and substitute correct path into file
SET( SEARCH_REGEX "%BUILD_DIR%" )
file( READ ${CMAKE_CURRENT_SOURCE_DIR}/setup.sh SETUP_FILE_TEXT )
STRING( REGEX REPLACE "${SEARCH_REGEX}" "${CMAKE_CURRENT_BINARY_DIR}" MODIFIED_FILE_TEXT "${SETUP_FILE_TEXT}" )
FILE( WRITE ${CMAKE_CURRENT_BINARY_DIR}/setup.sh "${MODIFIED_FILE_TEXT}" )

