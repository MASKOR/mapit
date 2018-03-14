################################################################################
#
# Copyright 2015-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
#           2015-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
#
################################################################################

#  This file is part of mapit.
#
#  Mapit is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  Mapit is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
#

## Using mapit in CMake projects can be done in 3 ways:
## 1) within one single CMake structure (core developer),
## 2) installed (users who add entitytypes and operators)
## 3) build directory (use mapit without installing it system wide) (this has to be added to CMAKE_PREFIX_PATH for importing project)
# use ${mapit_*} for compatibility of project structure (recommended)
# use mapit:: namespace only for exported/imported targets (users)
# use no prefix internally only (core developer)
# use find_package(mapit). It will make  ${mapit_*} variables available. The script detects your configuration and by using the
# variables CMakes dependecygraph will be set up correctly by pointing to either internal targets or imported targets.
# especially call find_package(mapit) in operators and layertypes and depend on the mapit::* targets used in your project.
# layertypes must export their targets. E.g. mapit_layertype_pointcloud2
# when depending on a layertype, find_package(layertype) must be called first
# the single mapitConfig.cmake configures all modules. This is a bit monolithic at the moment, but works.
# For the future, the configuration (logging enabled, pcl enabled, ...) should be exported when installed/built too.
# Preprocessor defines for included headers should not differ when building mapit (core developer) and using mapit.

# Altough this CMake is nearly perfect, there are some issues wich should be fixed in the future:
# 1) compile some operators/layertypes only if these frameworks are found
# 2) Output which frameworks were found and wich projects will/will not be built.
# 3) Remove additional path to configure boost, ...! Maybe add one path and rely on librarynames. PCL seems to break findBoost.
# 4) Make compilation type independent (Remove Cmake_Build_Type == Debug queries)

project(mapit)
cmake_minimum_required(VERSION 3.0.2) # for target_include_directories

function(message)
  list(GET ARGV 0 MessageType)
  if(MessageType STREQUAL FATAL_ERROR OR
     MessageType STREQUAL SEND_ERROR OR
     MessageType STREQUAL WARNING OR
     MessageType STREQUAL AUTHOR_WARNING)
    list(REMOVE_AT ARGV 0)
    _message(${MessageType} "${ARGV}")
  endif()
endfunction()

set(CPACK_PACKAGE_CONTACT "FH Aachen")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "d.bulla@fh-aachen.de")

set(CMAKE_CXX_STANDARD 11)

set(MAPIT_MAJOR_VERSION 0)
set(MAPIT_MINOR_VERSION 1)
set(MAPIT_PATCH_VERSION 0)
set(MAPIT_VERSION
  ${MAPIT_MAJOR_VERSION}.${MAPIT_MINOR_VERSION}.${MAPIT_PATCH_VERSION})

# This is used in the find script to determine, if targets non-imported projects can be used
#set(GLOBAL MAPIT_CMAKE_TARGETS_AVAILABLE TRUE)
set(MAPIT_CMAKE_TARGETS_AVAILABLE ON CACHE INTERNAL "Do not use, always true")

set(MAPIT_INSTALL_INCLUDE "mapit")
# include directory as subfolder. final form: #include <${MAPIT_INSTALL_INCLUDE}/${MAPIT_INSTALL_INCLUDE_LAYERTYPES}/*.h>
set(MAPIT_INSTALL_INCLUDE_ENTITYTYPES "entitytypes")
set(MAPIT_INSTALL_LAYERTYPES "mapit_entitytypes_") # prefix for shared libraries
set(MAPIT_INSTALL_OPERATORS "mapit_operators_") # prefix for shared libraries
add_definitions(-DMAPIT_INSTALL_ENTITYTYPES="${MAPIT_INSTALL_ENTITYTYPES}")
add_definitions(-DMAPIT_INSTALL_OPERATORS="${MAPIT_INSTALL_OPERATORS}")

if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
        message(WARNING "WARNING: In-Place Build not recommended. If you accidently commit and push wrong files to the repository, you owe a cup of coffee to your teammembers.")
        return(1) # stop messing up source directory if this happens accidently
endif()

# TODO: This is maybe not needed anymore
set(MAPIT_ROOT_DIR ${PROJECT_SOURCE_DIR})

# add custom cmake scripts. i.e. FindProcessorCount...
set(CMAKE_MODULE_PATH
    ${PROJECT_SOURCE_DIR}/cmake
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_MODULE_PATH}
)

if(WIN32)
    #globaly solve issue with windows.h and <algorithm>. Also fixes this for protobuf.
    add_definitions("-DNOMINMAX")
endif(WIN32)

# to make find_package(mapit) in build dir work
set(CMAKE_PREFIX_PATH
    ${PROJECT_SOURCE_DIR}/cmake
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_PREFIX_PATH}
)

##### let entitytypes and operators know what libraries are available #####
# Note: This could be part of the entitytypes cmake because all operators will depend on the entitytypes
# however at this point cmake already created the target for the operator and it might be hard to
# remove it again. Which entitytypes/operators are built is decided in a single space
# (layertypes_collection and operator_modules_collection)
set(MAPIT_ENABLE_PCL TRUE CACHE BOOL "Enable pcl Layers")
set(MAPIT_ENABLE_OPENVDB TRUE CACHE BOOL "Enable OpenVDB Layers")
set(MAPIT_ENABLE_OCTOMAP FALSE CACHE BOOL "Enable Octomap Layers")
set(MAPIT_ENABLE_TF TRUE CACHE BOOL "Enable Tf Layers, requiring Eigen2")

set(MAPIT_ENABLE_LOG4CPLUS FALSE CACHE BOOL "Enable logging with log4cplus. Uses standard c out as an alternative")
set(MAPIT_ENABLE_VIRTUALREALITY FALSE CACHE BOOL "Enable VR using OpenVR and Qt3dVirtualReality")

if(MAPIT_ENABLE_LOG4CPLUS)
  include(FindLog4cplus)
  if(LOG4CPLUS_FOUND)
    add_definitions(-DLOG4CPLUS_FOUND)
    add_definitions(-DMAPIT_ENABLE_LOG4CPLUS)
  endif(LOG4CPLUS_FOUND)
endif(MAPIT_ENABLE_LOG4CPLUS)


set(MAPIT_ENABLE_PCL TRUE CACHE BOOL "Enable pcl Layers")

#TODO: Is AUTOMOC still needed?
set(CMAKE_AUTOMOC ON)

include(FindProcessorCount)
message(STATUS "used number of processor cores: " ${PROCESSOR_COUNT})

include(CPack)

string( TOLOWER "${CMAKE_BUILD_TYPE}" LOWER_CMAKE_BUILD_TYPE )
if (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
    add_definitions("-DMAPIT_DEBUG")
endif()

if(MAPIT_ENABLE_LOG4CPLUS)
  include_directories(${LOG4CPLUS_INCLUDE_DIR})
endif(MAPIT_ENABLE_LOG4CPLUS)

add_subdirectory(libs/core)
add_subdirectory(libs/operator_modules_collection)
add_subdirectory(libs/mapit-msgs)
add_subdirectory(libs/interface_cpp)
add_subdirectory(libs/layertypes_collection)
add_subdirectory(libs/networking_node)
#add_subdirectory(libs/multi_repository)
add_subdirectory(libs/standard_repository_factory)
#add_subdirectory(externals)
#add_subdirectory(externals/yaml-cpp)
#add_subdirectory(tools/databasedump)
add_subdirectory(tools/checkout2filesystem)
add_subdirectory(tools/execute)
add_subdirectory(tools/checkout)
add_subdirectory(tools/log)
add_subdirectory(tools/commit)
add_subdirectory(tools/scripts)
add_subdirectory(tools/mapitd)
add_subdirectory(tools/commandlinetool)
add_subdirectory(tools/stream_to_ros)
add_subdirectory(tools/ls)

set(MAPIT_ENABLE_TESTS ON CACHE BOOL "Generate tests (need Qt)")
if(MAPIT_ENABLE_TESTS)
  add_subdirectory(test/unit_tests)
endif()

# json11 is a static library that is meant to be used in a shared library.
# Thus the cmake target itself is added and slightly reconfigured here
if(WIN32)
  add_subdirectory(externals/json11)
  set_property(TARGET json11 PROPERTY POSITION_INDEPENDENT_CODE 1)
endif(WIN32)

set(MAPIT_ENABLE_VISUALIZATION ON CACHE BOOL "Generate Visualization tools")

if(MAPIT_ENABLE_VISUALIZATION)
  add_subdirectory(tools/visualization)
  add_subdirectory(tools/gui)
  if(MAPIT_ENABLE_PCL)
    add_subdirectory(tools/Qt3DPointcloudRenderer)
  endif(MAPIT_ENABLE_PCL)
  add_subdirectory(tools/Qt3DOpenVDBRenderer)
  add_subdirectory(tools/graphblocks)
  if(MAPIT_ENABLE_VIRTUALREALITY)
    add_subdirectory(tools/visualization_vr)
  endif(MAPIT_ENABLE_VIRTUALREALITY)
endif(MAPIT_ENABLE_VISUALIZATION)


# a config file which can be included with find_package(mapit) and published targets mapit::core, mapit::interface_cpp, ...
configure_file(cmake/mapitConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/mapitConfig.cmake"
  @ONLY
)

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

set(ConfigPackageLocation lib/cmake/mapit)
install(
  FILES
    cmake/mapitConfig.cmake
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
