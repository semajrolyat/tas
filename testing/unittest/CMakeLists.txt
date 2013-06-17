

add_executable( pdtrajectories pdtrajectories.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( pdtrajectories ${TAS_LIBRARIES} )

add_executable( cpuinfo cpuinfo.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( cpuinfo ${TAS_LIBRARIES} )
