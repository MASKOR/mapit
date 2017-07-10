 # - Config file for the mapit package, loads all mapit target mapit::core, mapit::interface, mapit::interface_cpp
 #NOTE ON find_package(mapit):
 # use add_library(<name> mapit::interface) if the target is imported
 # use add_library(<name> interface) if the target is only used in the mapit-cmake project
 # use add_library(<name> ${mapit_interface}) to make both work.
 # use other build system for a perfect project setup

# Compute paths
#get_filename_component(MAPIT_CORE_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(MAPIT_CORE_CMAKE_DIR ${CMAKE_CURRENT_LIST_DIR})
#set(MAPIT_CORE_INCLUDE_DIRS "@CONF_INCLUDE_DIRS@")

# not supported with find script or configuration at the moment
# TODO: Export the exact build configuration to Config-Script.
set(WITH_PCL TRUE CACHE BOOL "Enable pcl Layers")
set(WITH_OPENVDB TRUE CACHE BOOL "Enable OpenVDB Layers")
set(WITH_OCTOMAP TRUE CACHE BOOL "Enable Octomap Layers")
set(WITH_TF TRUE CACHE BOOL "Enable Tf Layers, requiring Eigen2")

set(WITH_LOG4CPLUS FALSE CACHE BOOL "Enable logging with log4cplus. Uses standard c out as an alternative")

if(MAPIT_CMAKE_TARGETS_AVAILABLE)
  message("mapit found in cmake project. Using local targets, dependencies will recompile on demand.")
  set(mapit_core core)
  set(mapit_interface interface)
  set(mapit_interface_cpp interface_cpp)
  set(mapit_standard_repository_factory standard_repository_factory)
  set(mapit_networking_node networking_node)
  if(WITH_PCL)
    set(mapit_layertype_pointcloud2 layertype_pointcloud2)
  endif(WITH_PCL)
else(MAPIT_CMAKE_TARGETS_AVAILABLE)
  message("mapit targets are IMPORTED (installed or in build dir). Please use mapit::* or \${mapit_*} to refer to mapit targets.")
  # TODO: installed directory structure
  # only used if in build directory
  set(CMAKE_PREFIX_PATH
    ${CMAKE_CURRENT_LIST_DIR}/libs/core
    ${CMAKE_CURRENT_LIST_DIR}/libs/upns_interface
    ${CMAKE_CURRENT_LIST_DIR}/libs/upns_interface_cpp
    ${CMAKE_CURRENT_LIST_DIR}/libs/standard_repository_factory
    ${CMAKE_PREFIX_PATH}
  )

  include(CMakeFindDependencyMacro)
  #find_dependency can not do components yet
  find_dependency(Boost REQUIRED)
  find_dependency(mapit_core REQUIRED)
  find_dependency(mapit_interface REQUIRED)
  find_dependency(mapit_interface_cpp REQUIRED)
  find_dependency(mapit_standard_repository_factory REQUIRED)
  find_dependency(mapit_networking_node REQUIRED)
  set(mapit_core mapit::core)
  set(mapit_interface mapit::interface)
  set(mapit_interface_cpp mapit::interface_cpp)
  set(mapit_standard_repository_factory mapit::standard_repository_factory)
  set(mapit_networking_node mapit::networking_node)
  if(WITH_PCL)
    set(CMAKE_PREFIX_PATH
      ${CMAKE_CURRENT_LIST_DIR}/libs/layertypes_collection/pointcloud2
      ${CMAKE_PREFIX_PATH}
    )
    find_dependency(mapit_layertype_pointcloud2 REQUIRED)
    set(mapit_layertype_pointcloud2 mapit::layertype_pointcloud2)
  endif(WITH_PCL)
endif(MAPIT_CMAKE_TARGETS_AVAILABLE)
# Our library dependencies (contains definitions for IMPORTED targets)
#if(NOT TARGET upns_core AND NOT upns_core_BINARY_DIR)
#  include("${MAPIT_CORE_CMAKE_DIR}/mapitCoreTargets.cmake")
  #include("${MAPIT_CORE_CMAKE_DIR}/mapitCoreMacros.cmake")
#endif()
 
# These are IMPORTED targets created by mapitCoreTargets.cmake
#set(MAPIT_CORE_LIBRARIES upns_core)
