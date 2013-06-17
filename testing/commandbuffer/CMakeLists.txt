add_executable( commandbuffer_controller controller.cpp ${TAS_HEADERS} CommandBuffer.h DynamicsPlugin.h ${TAS_SOURCES} )
target_link_libraries( commandbuffer_controller ${TAS_LIBRARIES} )

add_library( pendulum-pd-controller "MODULE" controller.cpp )
target_link_libraries( pendulum-pd-controller ${MOBY_LIBRARIES} )

add_executable( commandbuffer_coordinator coordinator.cpp ${TAS_HEADERS} CommandBuffer.h DynamicsPlugin.h ${TAS_SOURCES} )
target_link_libraries( commandbuffer_coordinator ${TAS_LIBRARIES} )

add_library( commandbuffer_dynamics "MODULE" dynamics.cpp ${TAS_HEADERS} CommandBuffer.h DynamicsPlugin.h ${TAS_SOURCES} )
target_link_libraries( commandbuffer_dynamics ${TAS_LIBRARIES} )

add_executable( commandbuffer_server commandbuffer_server.cpp ${TAS_HEADERS} CommandBuffer.h ${TAS_SOURCES} )
target_link_libraries( commandbuffer_server ${TAS_LIBRARIES} )

add_executable( commandbuffer_client commandbuffer_client.cpp ${TAS_HEADERS} CommandBuffer.h ${TAS_SOURCES} )
target_link_libraries( commandbuffer_client ${TAS_LIBRARIES} )

add_subdirectory( "example" )
