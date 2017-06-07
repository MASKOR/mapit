 # - Config file for the mapit::core package
# It defines the following variables
#  MAPIT_CORE_INCLUDE_DIRS - include directories for mapit::core
#  MAPIT_CORE_LIBRARIES    - libraries to link against
 
# Compute paths
#get_filename_component(MAPIT_CORE_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
#set(MAPIT_CORE_INCLUDE_DIRS "@CONF_INCLUDE_DIRS@")
 
include(CMakeFindDependencyMacro)
find_dependency(Boost REQUIRED COMPONENTS filesystem iostreams)

# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET upns_core AND NOT upns_core_BINARY_DIR)
  include("${MAPIT_CORE_CMAKE_DIR}/mapitCoreTargets.cmake")
  include("${MAPIT_CORE_CMAKE_DIR}/mapitCoreMacros.cmake")
endif()
 
# These are IMPORTED targets created by mapitCoreTargets.cmake
#set(MAPIT_CORE_LIBRARIES upns_core)
