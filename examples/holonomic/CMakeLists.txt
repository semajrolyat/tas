add_executable( controller ${CMAKE_CURRENT_SOURCE_DIR}/controller.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( controller ${TAS_LIBRARIES} )

add_executable( planner ${CMAKE_CURRENT_SOURCE_DIR}/planner.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( planner ${TAS_LIBRARIES} )

add_executable( coordinator ${CMAKE_CURRENT_SOURCE_DIR}/coordinator.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( coordinator ${TAS_LIBRARIES} )

add_library( dynamics "MODULE" ${CMAKE_CURRENT_SOURCE_DIR}/dynamics.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( dynamics ${TAS_LIBRARIES} )

file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/ship.xml DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
