 # - Config file for the mapit::core package
# It defines the following variables
#  MAPIT_CORE_INCLUDE_DIRS - include directories for mapit::core
#  MAPIT_CORE_LIBRARIES    - libraries to link against
 
# Compute paths
#get_filename_component(MAPIT_CORE_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(MAPIT_CORE_CMAKE_DIR ${CMAKE_CURRENT_LIST_DIR})
#set(MAPIT_CORE_INCLUDE_DIRS "@CONF_INCLUDE_DIRS@")

include(CMakeFindDependencyMacro)
#find_dependency can not do components yet
#find_dependency(Boost REQUIRED COMPONENTS filesystem iostreams)
find_dependency(Boost REQUIRED)
find_dependency(mapit_interface_cpp REQUIRED)
find_dependency(mapit_core REQUIRED)
# conditional compiling for different variables set/unset (like MAPIT_ENABLE_LOG4CPLUS) not supported.
#if($<MAPIT_ENABLE_LOG4CPLUS>)
#  include(FindLog4cplus)
#  if(LOG4CPLUS_FOUND)
#    add_definitions(-DLOG4CPLUS_FOUND)
#  endif(LOG4CPLUS_FOUND)
#endif($<MAPIT_ENABLE_LOG4CPLUS>)


# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET mapit::networking_node AND NOT mapit::networking_node_BINARY_DIR)
  include("${MAPIT_CORE_CMAKE_DIR}/mapit_networking_nodeTargets.cmake")
  #include("${MAPIT_CORE_CMAKE_DIR}/mapitCoreMacros.cmake")
endif()
 
# These are IMPORTED targets created by mapitCoreTargets.cmake
#set(MAPIT_CORE_LIBRARIES mapit_core)
