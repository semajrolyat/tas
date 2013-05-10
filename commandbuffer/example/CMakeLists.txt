add_executable( shm_server shm_server.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( shm_server ${TAS_LIBRARIES} )

add_executable( shm_client shm_client.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( shm_client ${TAS_LIBRARIES} )

