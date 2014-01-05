add_executable( controller ${CMAKE_CURRENT_SOURCE_DIR}/controller.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( controller ${TAS_LIBRARIES} )

add_executable( coordinator ${CMAKE_CURRENT_SOURCE_DIR}/coordinator.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( coordinator ${TAS_LIBRARIES} )

add_library( dynamics "MODULE" ${CMAKE_CURRENT_SOURCE_DIR}/dynamics.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( dynamics ${TAS_LIBRARIES} )

#add_executable( ${CMAKE_CURRENT_BINARY_DIR}/controller ${CMAKE_CURRENT_SOURCE_DIR}/controller.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
#target_link_libraries( ${CMAKE_CURRENT_BINARY_DIR}/controller ${TAS_LIBRARIES} )

#add_executable( ${CMAKE_CURRENT_BINARY_DIR}/coordinator ${CMAKE_CURRENT_SOURCE_DIR}/coordinator.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
#target_link_libraries( ${CMAKE_CURRENT_BINARY_DIR}/coordinator ${TAS_LIBRARIES} )

#add_library( ${CMAKE_CURRENT_BINARY_DIR}/dynamics "MODULE" ${CMAKE_CURRENT_SOURCE_DIR}/dynamics.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
#target_link_libraries( ${CMAKE_CURRENT_BINARY_DIR}/dynamics ${TAS_LIBRARIES} )


