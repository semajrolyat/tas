find_package( ODE REQUIRED )

include_directories( ${ODE_INCLUDE_DIR} )

#add_executable( demo_plane2d ${CMAKE_CURRENT_SOURCE_DIR}/demo_plane2d.cpp )
#target_link_libraries( demo_plane2d ${ODE_LIBRARY} )

add_executable( demo_ode ${CMAKE_CURRENT_SOURCE_DIR}/demo_ode.cpp )
target_link_libraries( demo_ode ${ODE_LIBRARY} )

add_executable( tas_pen ${CMAKE_CURRENT_SOURCE_DIR}/tas_pen.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( tas_pen ${ODE_LIBRARY} ${TAS_LIBRARIES} )
