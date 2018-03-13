################################################################################
#
# Copyright      2017 Marcus Meeßen	<marcus.meessen@alumni.fh-aachen.de>
#                2017 Tobias Neumann	<t.neumann@fh-aachen.de>
#                2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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
    cmake_minimum_required(VERSION 3.0)

    # describe operator
    set(OPERATOR_NAME filter_radius_outlier_removal)
    set(OPERATOR_VERSION 1.0)
    project(${OPERATOR_NAME})
    include_directories(${PROJECT_SOURCE_DIR}/src/)
    add_definitions(-DOPERATOR_NAME="${OPERATOR_NAME}")
    add_definitions(-DOPERATOR_VERSION=${OPERATOR_VERSION})

    # pcl
    find_package(PCL QUIET 1.8 REQUIRED
        COMPONENTS filters)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

    # Qt
    find_package(Qt5Core REQUIRED)
    find_package(Qt5Gui REQUIRED)

    # mapit
    find_package(mapit REQUIRED)

    # Qt
    find_package(Qt5Core REQUIRED)

    # describe target
    file(GLOB_RECURSE SOURCES
        ${PROJECT_SOURCE_DIR}/src/*.cpp)
    file(GLOB_RECURSE HEADERS
        ${PROJECT_SOURCE_DIR}/src/*.hpp)
    add_library(${PROJECT_NAME} SHARED
        ${SOURCES}
        ${HEADERS})
    set_target_properties(${PROJECT_NAME} PROPERTIES
        VERSION ${OPERATOR_VERSION}
        SOVERSION ${OPERATOR_VERSION}
        PREFIX ${CMAKE_SHARED_LIBRARY_PREFIX}${MAPIT_INSTALL_OPERATORS})

    # linker
    target_link_libraries(${PROJECT_NAME}
        ${PCL_FILTERS_LIBRARIES}
        ${mapit_interface}
        ${mapit_core}
        layertype_pointcloud2)

    qt5_use_modules(${PROJECT_NAME} Core Gui)

    install(TARGETS ${PROJECT_NAME}
        DESTINATION lib)
endif(NOT DISABLE_PCL)
