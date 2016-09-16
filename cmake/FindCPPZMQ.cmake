 # - Try to find ZMQ
# Once done this will define
# CPPZMQ_FOUND - System has CPPZMQ
# CPPZMQ_INCLUDE_DIRS - The ZMQ include directories
# CPPZMQ_DEFINITIONS - Compiler switches required for using ZMQ

find_path ( CPPZMQ_INCLUDE_DIR zmq.hpp HINTS ../externals/cppzmq )

set ( CPPZMQ_INCLUDE_DIRS ${CPPZMQ_INCLUDE_DIR} )

include ( FindPackageHandleStandardArgs )
# handle the QUIETLY and REQUIRED arguments and set ZMQ_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args ( CPPZMQ DEFAULT_MSG CPPZMQ_INCLUDE_DIR )
