 # - Config file for the mapit::core package
# It defines the following variables
#  MAPIT_INTERFACE_CPP_INCLUDE_DIRS - include directories for mapit::core
#  MAPIT_INTERFACE_CPP_LIBRARIES    - libraries to link against
 
# Compute paths
#get_filename_component(MAPIT_INTERFACE_CPP_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(MAPIT_INTERFACE_CPP_CMAKE_DIR ${CMAKE_CURRENT_LIST_DIR})
#set(MAPIT_INTERFACE_CPP_INCLUDE_DIRS "@CONF_INCLUDE_DIRS@")
 
include(CMakeFindDependencyMacro)
find_dependency(mapit_interface_cpp REQUIRED)
find_dependency(mapit_core REQUIRED)
find_dependency(mapit_networking_node REQUIRED)

# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET mapit::standard_repository_factory AND NOT mapit::standard_repository_factory_BINARY_DIR)
  include("${MAPIT_INTERFACE_CPP_CMAKE_DIR}/mapit_standard_repository_factoryTargets.cmake")
endif()

# These are IMPORTED targets created by mapitInterfaceCppTargets.cmake
#set(MAPIT_INTERFACE_CPP_LIBRARIES upns_interface_cpp)
