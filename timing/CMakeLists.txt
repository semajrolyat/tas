add_executable( timing_controller controller.cpp ${TAS_HEADERS} ActuatorMessage.h ${TAS_SOURCES} )
target_link_libraries( timing_controller ${TAS_LIBRARIES} )

add_executable( timing_coordinator coordinator.cpp ${TAS_HEADERS} ActuatorMessage.h DynamicsPlugin.h ${TAS_SOURCES} )
target_link_libraries( timing_coordinator ${TAS_LIBRARIES} )

add_library( timing_dynamics "MODULE" dynamics.cpp ${TAS_HEADERS} ActuatorMessage.h DynamicsPlugin.h ${TAS_SOURCES} )
target_link_libraries( timing_dynamics ${TAS_LIBRARIES} )
