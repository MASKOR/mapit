################################################################################
#
# Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
#           2015-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

if(NOT DISABLE_PCL)
project(normalestimation)
cmake_minimum_required(VERSION 2.8)

#SET(CMAKE_CXX_FLAGS "-std=c++98")
#set (CMAKE_CXX_STANDARD 98)

set(OPERATOR_VERSION 1)

find_package(mapit REQUIRED)

aux_source_directory(. SRC_LIST)

add_definitions(-DOPERATOR_NAME="${PROJECT_NAME}")

add_definitions(-DOPERATOR_VERSION=${OPERATOR_VERSION})

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

find_package(PCL QUIET REQUIRED COMPONENTS common io segmentation sample_consensus filters kdtree search)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB_RECURSE OPERATOR_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)
add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${OPERATOR_SOURCES} )
target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    ${PCL_SEARCH_LIBRARIES}
    ${PCL_KDTREE_LIBRARIES}
    ${mapit_interface}
    ${mapit_core}
    json11
    layertype_pointcloud2)

SET_TARGET_PROPERTIES(
    ${PROJECT_NAME}
  PROPERTIES
    VERSION ${OPERATOR_VERSION}
    SOVERSION ${OPERATOR_VERSION}
)

if(UNIX)
    # tell CMake Prefix is "lib" otherwise it can not be queried
    set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib")
endif(UNIX)
get_target_property(OLD_PREFIX ${PROJECT_NAME} PREFIX)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX ${OLD_PREFIX}${MAPIT_INSTALL_OPERATORS})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

endif(NOT DISABLE_PCL)
