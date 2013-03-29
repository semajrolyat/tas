# On success, the macro sets the following variables:
# MOBY_FOUND       = if the library found
# MOBY_LIBRARIES   = full path to the library
# MOBY_INCLUDE_DIR = where to find the library headers

add_definitions (-DBUILD_DOUBLE)

set(MOBY_INCLUDE_DIR /home/james/Moby/include)
set(MOBY_LIBRARIES /home/james/Moby/build/libMoby.so)
