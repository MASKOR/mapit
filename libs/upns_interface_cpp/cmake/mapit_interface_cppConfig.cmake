 # - Config file for the mapit::core package
# It defines the following variables
#  MAPIT_INTERFACE_CPP_INCLUDE_DIRS - include directories for mapit::core
#  MAPIT_INTERFACE_CPP_LIBRARIES    - libraries to link against
 
# Compute paths
#get_filename_component(MAPIT_INTERFACE_CPP_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(MAPIT_INTERFACE_CPP_CMAKE_DIR ${CMAKE_CURRENT_LIST_DIR})
#set(MAPIT_INTERFACE_CPP_INCLUDE_DIRS "@CONF_INCLUDE_DIRS@")
 
include(CMakeFindDependencyMacro)
find_dependency(mapit_interface REQUIRED)

# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET mapit::interface_cpp AND NOT mapit::interface_cpp_BINARY_DIR)
  include("${MAPIT_INTERFACE_CPP_CMAKE_DIR}/mapit_interface_cppTargets.cmake")
endif()
 
set(MAPIT_INSTALL_ENTITYTYPES "upns_layertypes_") # prefix for shared libraries
set(MAPIT_INSTALL_OPERATORS "upns_operators_") # prefix for shared libraries

# These are IMPORTED targets created by mapitInterfaceCppTargets.cmake
#set(MAPIT_INTERFACE_CPP_LIBRARIES upns_interface_cpp)
