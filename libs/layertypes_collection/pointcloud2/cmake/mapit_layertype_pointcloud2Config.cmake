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
find_dependency(PCL REQUIRED)
find_dependency(mapit_interface_cpp REQUIRED)
find_dependency(mapit_core REQUIRED)

# TODO: is that best practice?
# find a way to evaluate PCL_INCLUDE_DIRS at configure time of the DEPENDING project.
# Reason: Variable evaluates to different values depending on where pcl is installed
#target_include_directories(mapit::layertype_pointcloud2 ${PCL_INCLUDE_DIRS})

# conditional compiling for different variables set/unset (like WITH_LOG4CPLUS) not supported.
#if($<WITH_LOG4CPLUS>)
#  include(FindLog4cplus)
#  if(LOG4CPLUS_FOUND)
#    add_definitions(-DLOG4CPLUS_FOUND)
#  endif(LOG4CPLUS_FOUND)
#endif($<WITH_LOG4CPLUS>)


# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET mapit::layertype_pointcloud2 AND NOT mapit::layertype_pointcloud2_BINARY_DIR)
  include("${MAPIT_CORE_CMAKE_DIR}/mapit_layertype_pointcloud2Targets.cmake")
  #include("${MAPIT_CORE_CMAKE_DIR}/mapitCoreMacros.cmake")
endif()
 
# These are IMPORTED targets created by mapitCoreTargets.cmake
#set(MAPIT_CORE_LIBRARIES upns_core)
