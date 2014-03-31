add_executable( test-timer ${CMAKE_CURRENT_SOURCE_DIR}/test_timer.cpp ${PURSUIT_HEADERS} ${PURSUIT_SOURCES} )
target_link_libraries( test-timer ${PURSUIT_LIBRARIES} )

add_executable( test-heap ${CMAKE_CURRENT_SOURCE_DIR}/test_heap.cpp ${PURSUIT_HEADERS} ${PURSUIT_SOURCES} )
target_link_libraries( test-heap ${PURSUIT_LIBRARIES} )

add_executable( test-log ${CMAKE_CURRENT_SOURCE_DIR}/test_log.cpp ${PURSUIT_HEADERS} ${PURSUIT_SOURCES} )
target_link_libraries( test-log ${PURSUIT_LIBRARIES} )

add_executable( test-coordinator ${CMAKE_CURRENT_SOURCE_DIR}/test_coordinator.cpp ${PURSUIT_HEADERS} ${PURSUIT_SOURCES} )
target_link_libraries( test-coordinator ${PURSUIT_LIBRARIES} )

add_executable( client-process ${CMAKE_CURRENT_SOURCE_DIR}/client_process.cpp ${PURSUIT_HEADERS} ${PURSUIT_SOURCES} )
target_link_libraries( client-process ${PURSUIT_LIBRARIES} )
