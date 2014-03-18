add_executable( test-timer ${CMAKE_CURRENT_SOURCE_DIR}/test_timer.cpp ${PURSUIT_HEADERS} ${PURSUIT_SOURCES} )
target_link_libraries( test-timer ${PURSUIT_LIBRARIES} )

