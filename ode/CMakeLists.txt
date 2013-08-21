find_package( ODE REQUIRED )

include_directories( ${ODE_INCLUDE_DIR} )

add_executable( tas_pen ${CMAKE_CURRENT_SOURCE_DIR}/tas_pen.cpp ${TAS_HEADERS} ${TAS_SOURCES} )
target_link_libraries( tas_pen ${ODE_LIBRARY} ${TAS_LIBRARIES} )
