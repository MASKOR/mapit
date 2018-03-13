################################################################################
#
# Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
#           2015-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
#                2017 Kai Krueckel	<kai.krueckel@alumni.fh-aachen.de>
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

project(moving_least_squares)
cmake_minimum_required(VERSION 2.8)

set(OPERATOR_VERSION 1)

find_package(mapit REQUIRED)

aux_source_directory(. SRC_LIST)

add_definitions(-DOPERATOR_NAME="${PROJECT_NAME}")
add_definitions(-DOPERATOR_VERSION=${OPERATOR_VERSION})

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

set(CMAKE_AUTOMOC ON)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io filters search octree kdtree search surface)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(PCL_REALLY_ALL_LIBRARIES ${PROTOBUF_LIBRARIES} ${PCL_LIBRARIES}
                                                   ${PCL_COMMON_LIBRARIES}
                                                   ${PCL_IO_LIBRARIES}
                                                   ${PCL_FILTERS_LIBRARIES})

file(GLOB_RECURSE UPNS_OPERATOR_MLS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)
add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${UPNS_OPERATOR_MLS_SOURCES})

target_link_libraries( ${PROJECT_NAME} ${PCL_REALLY_ALL_LIBRARIES} ${mapit_interface} ${mapit_core} layertype_pointcloud2)

qt5_use_modules(${PROJECT_NAME} Core Gui)

SET_TARGET_PROPERTIES(
    ${PROJECT_NAME}
  PROPERTIES
    VERSION ${OPERATOR_VERSION}
    SOVERSION ${OPERATOR_VERSION}
)

#get_property(lib_location TARGET ${PROJECT_NAME} PROPERTY LOCATION)

if (CMAKE_HOST_UNIX)
    set(FILENAME_PREFIX "lib")
    set(FILENAME_POSTFIX ".so")
else (CMAKE_HOST_UNIX)
    set(FILENAME_PREFIX "")
    set(FILENAME_POSTFIX ".dll")
endif (CMAKE_HOST_UNIX)

#set(LIB_FILENAME ${FILENAME_PREFIX}${PROJECT_NAME}${CMAKE_DEBUG_POSTFIX}${FILENAME_POSTFIX})
#set(LIB_FILENAME_VERSION ${FILENAME_PREFIX}${PROJECT_NAME}${CMAKE_DEBUG_POSTFIX}${FILENAME_POSTFIX}.${OPERATOR_VERSION})

#add_custom_command(
#    TARGET ${PROJECT_NAME} POST_BUILD
#    COMMAND ln -sf ${LIB_FILENAME} ${LIB_FILENAME_VERSION} WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH}
#    COMMAND ln -sf ${LIB_FILENAME} ${LIB_FILENAME_VERSION} WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH}
#)
if(UNIX)
    # tell CMake Prefix is "lib" otherwise it can not be queried
    set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib")
endif(UNIX)
get_target_property(OLD_PREFIX ${PROJECT_NAME} PREFIX)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX ${OLD_PREFIX}${UPNS_INSTALL_OPERATORS})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
