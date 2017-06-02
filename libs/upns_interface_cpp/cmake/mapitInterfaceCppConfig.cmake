 # - Config file for the mapit::core package
# It defines the following variables
#  MAPIT_INTERFACE_CPP_INCLUDE_DIRS - include directories for mapit::core
#  MAPIT_INTERFACE_CPP_LIBRARIES    - libraries to link against
 
# Compute paths
#get_filename_component(MAPIT_INTERFACE_CPP_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
#set(MAPIT_INTERFACE_CPP_INCLUDE_DIRS "@CONF_INCLUDE_DIRS@")
 
# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET upns_interface_cpp AND NOT upns_interface_cpp_BINARY_DIR)
  include("${MAPIT_INTERFACE_CPP_CMAKE_DIR}/mapitInterfaceCppTargets.cmake")
endif()
 
# These are IMPORTED targets created by mapitInterfaceCppTargets.cmake
#set(MAPIT_INTERFACE_CPP_LIBRARIES upns_interface_cpp)
