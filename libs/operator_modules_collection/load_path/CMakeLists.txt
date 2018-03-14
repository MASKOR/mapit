################################################################################
#
# Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
#           2015-2016 Tobias Neumann	<t.neumann@fh-aachen.de>
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

project(load_path)
cmake_minimum_required(VERSION 2.8)

set(OPERATOR_VERSION 1)

find_package(mapit REQUIRED)

aux_source_directory(. SRC_LIST)

add_definitions(-DOPERATOR_NAME="${PROJECT_NAME}")

add_definitions(-DOPERATOR_VERSION=${OPERATOR_VERSION})

file(GLOB_RECURSE OPERATOR_LOAD_PATH_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

set(CMAKE_AUTOMOC ON)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${OPERATOR_LOAD_PATH_SOURCES} )
target_link_libraries( ${PROJECT_NAME} ${mapit_interface} ${mapit_core} layertype_pose_path)

qt5_use_modules(${PROJECT_NAME} Core Gui)

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
