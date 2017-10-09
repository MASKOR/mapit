set(MAPIT_CORE_CMAKE_DIR ${CMAKE_CURRENT_LIST_DIR})

include(CMakeFindDependencyMacro)
find_dependency(mapit_interface_cpp REQUIRED)
find_dependency(mapit_core REQUIRED)
# conditional compiling for different variables set/unset (like WITH_LOG4CPLUS) not supported.
#if($<WITH_LOG4CPLUS>)
#  include(FindLog4cplus)
#  if(LOG4CPLUS_FOUND)
#    add_definitions(-DLOG4CPLUS_FOUND)
#  endif(LOG4CPLUS_FOUND)
#endif($<WITH_LOG4CPLUS>)


# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET mapit::layertype_tf AND NOT mapit::layertype_tf_BINARY_DIR)
  include("${MAPIT_CORE_CMAKE_DIR}/mapit_layertype_tfTargets.cmake")
endif()
