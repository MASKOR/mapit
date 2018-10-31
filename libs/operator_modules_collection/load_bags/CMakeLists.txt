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
  cmake_minimum_required(VERSION 2.8)
  project(load_bags)

  # message colors
  if(NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(Red         "${Esc}[31m")
    set(Green       "${Esc}[32m")
  endif()

  # required, but disables this project without an error
  find_package(rosbag)
  find_package(sensor_msgs)
  find_package(pcl_conversions)

  if(NOT (rosbag_FOUND AND sensor_msgs_FOUND AND pcl_conversions_FOUND))
    message(AUTHOR_WARNING "${Red}"
      "Operator \"${PROJECT_NAME}\" will not be build, dependencies are not satisfied"
      "\n - rosbag: ${rosbag_FOUND}"
      "\n - sensor_msgs: ${sensor_msgs_FOUND}"
      "\n - pcl_conversions: ${pcl_conversions_FOUND}"
      "${ColourReset}")
  else()
    message(AUTHOR_WARNING "${Green}"
      "Operator \"${PROJECT_NAME}\" will be build"
      "${ColourReset}")

    set(OPERATOR_VERSION 1)
    add_definitions(-DOPERATOR_NAME="${PROJECT_NAME}")
    add_definitions(-DOPERATOR_VERSION=${OPERATOR_VERSION})

    # describe target
    file(GLOB_RECURSE OPERATOR_SOURCES
      ${PROJECT_SOURCE_DIR}/src/*.cpp
      ${PROJECT_SOURCE_DIR}/src/*.h
      )
    add_library(${PROJECT_NAME} SHARED
      ${SRC_LIST}
      ${OPERATOR_SOURCES}
      )
    set_target_properties(${PROJECT_NAME} PROPERTIES
      VERSION ${OPERATOR_VERSION}
      SOVERSION ${OPERATOR_VERSION}
      CXX_STANDARD 14
      )
    target_include_directories(${PROJECT_NAME}
      PRIVATE
        ${PROJECT_SOURCE_DIR}/src/
      )

    # mapit
    find_package(mapit REQUIRED)
    target_link_libraries(${PROJECT_NAME}
      ${mapit_core}
      ${mapit_interface}
      layertype_pointcloud2
      layertype_tf
      )

    # Qt
    set(CMAKE_AUTOMOC ON)
    find_package(Qt5Core REQUIRED)
    qt5_use_modules(${PROJECT_NAME} Core)

    # ROS
    #find_package(rosbag REQUIRED)
    #find_package(sensor_msgs REQUIRED)
    target_include_directories(${PROJECT_NAME} PRIVATE
      ${rosbag_INCLUDE_DIRS}
      ${sensor_msgs_INCLUDE_DIRS}
      ${pcl_conversions_INCLUDE_DIRS}
      )
    target_link_libraries(${PROJECT_NAME}
      ${rosbag_LIBRARIES}
      ${sensor_msgs_LIBRARIES}
      ${pcl_conversions_LIBRARIES}
      )

    SET_TARGET_PROPERTIES(
        ${PROJECT_NAME}
      PROPERTIES
        VERSION ${OPERATOR_VERSION}
        SOVERSION ${OPERATOR_VERSION}
    )

    # naming and installing
    if(UNIX)
        # tell CMake Prefix is "lib" otherwise it can not be queried
        set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib")
    endif(UNIX)
    get_target_property(OLD_PREFIX ${PROJECT_NAME} PREFIX)
    set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX ${OLD_PREFIX}${MAPIT_INSTALL_OPERATORS})

    install(TARGETS ${PROJECT_NAME} DESTINATION lib)

  endif()
endif(NOT DISABLE_PCL)
