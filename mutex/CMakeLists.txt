add_executable( mutex_proc1 proc1.cpp Buffer.h )
target_link_libraries( mutex_proc1 ${TAS_LIBRARIES} )

add_executable( mutex_proc2 proc2 Buffer.h )
target_link_libraries( mutex_proc2 ${TAS_LIBRARIES} )

