add_executable( timing_controller controller.cpp TAS.h ActuatorMessage.h )
target_link_libraries( timing_controller ${TAS_LIBRARIES} )

add_executable( timing_coordinator coordinator.cpp TAS.h ActuatorMessage.h DynamicsPlugin.h )
target_link_libraries( timing_coordinator ${TAS_LIBRARIES} )

add_library( timing_dynamics "MODULE" dynamics.cpp TAS.h ActuatorMessage.h DynamicsPlugin.h )
target_link_libraries( timing_dynamics ${TAS_LIBRARIES} )

#add_executable( thread_test thread_test.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
#target_link_libraries( thread_test ${TAS_LIBRARIES} )

add_executable( priority_test priority_test.cpp TAS.h )
target_link_libraries( priority_test ${TAS_LIBRARIES} )

#add_executable( actuatormessage_test actuatormessage_test.cpp TAS.h )
#target_link_libraries( actuatormessage_test ${TAS_LIBRARIES} )
