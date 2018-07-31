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

project(layertype_pointcloud2)

cmake_minimum_required(VERSION 2.8)
#find_package(PCL QUIET REQUIRED COMPONENTS common io apps filters kdtree search features surface keypoints registration)

find_package(mapit REQUIRED)

set (CMAKE_CXX_STANDARD 11)

find_package(Boost COMPONENTS serialization system filesystem date_time)

find_package(Protobuf)
include_directories(${PROTOBUF_INCLUDE_DIRS})

find_package(PCL QUIET REQUIRED COMPONENTS common io)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Eigen3 Will just not be found on windows
if(WIN32)
    find_package(Eigen3)
    set(EIGEN3_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/externals/eigen3) # contains eigen3/Eigen/...
else(WIN32)
    find_package(Eigen3 REQUIRED)
endif(WIN32)

# TODO: If language could not be determined: There are Additional cmake variable for boost that must be set. Fix this
file(GLOB_RECURSE LAYERTYPE_POINTCLOUD2_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/src/*.h
                                                ${PROJECT_SOURCE_DIR}/include/*.h )
add_library(${PROJECT_NAME} SHARED ${LAYERTYPE_POINTCLOUD2_SOURCES} )
target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES} ${PCL_LIBRARIES} ${mapit_interface_cpp})

# About dependency to pcl
# PCL is not known to have the newest and cleanes CMake files.
# Thus there are variables like PCL_INCLUDE_DIRS
# When using the layertype, we automatically introduce a dependency to pcl
# CMake can add the dependency by a multitude of ways.
# 1. If the project is in the cmake project hierarchy, everything is ok (no futher details here)
# 2. When mapit is installed, the variables PCL_INCLUDE_DIRS can not be set automatically, because
#    the path will differ from what the installation directory is.
#    But it is possible to find_dependecy(PCL) and re-read the variable.
#    This must be done in a config or targets script.
#    The targts script is generated automatically using the new generator expressions.
#    TODO: find a way the re-evaluate variables in generator expressions. Until then, option
#      1) projects using this layertype must include PCL on their own (only header). Or
#      2) installation env must be equal to dev env by executing e.g.:
#      ln -s /usr/include /usr/local/include
#      ln -s /usr/lib/libpcl_* /usr/local/
#      (2) is current workaround)
#    Note: just putting the below "private" varibale to "public" will not do the job (tested after installation on other system with mapit_samples).

target_include_directories(${PROJECT_NAME} PUBLIC  ${PCL_INCLUDE_DIRS}
                                                  #$<BUILD_INTERFACE:${PCL_INCLUDE_DIRS}>
                                                   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                   $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME} PRIVATE PROJECT_NAME="${PROJECT_NAME}"
                                           PUBLIC  PointcloudEntitydata_TYPENAME="${PROJECT_NAME}"
                                                   MAPIT_ENABLE_PCL=1)

if(UNIX)
  # tell CMake Prefix is "lib" otherwise it can not be queried
  set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib")
endif(UNIX)
get_target_property(OLD_PREFIX ${PROJECT_NAME} PREFIX)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX ${OLD_PREFIX}${MAPIT_INSTALL_ENTITYTYPES})

#install(TARGETS ${PROJECT_NAME} DESTINATION lib)
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${CMAKE_LIBRARY_ARCHITECTURE} )


include(GenerateExportHeader)

generate_export_header(${PROJECT_NAME})
#set_property(TARGET ${PROJECT_NAME} PROPERTY VERSION ${MAPIT_VERSION})
#set_property(TARGET ${PROJECT_NAME} PROPERTY SOVERSION ${MAPIT_VERSION})
set_property(TARGET ${PROJECT_NAME} PROPERTY
  INTERFACE_mapit_layertype_pointcloud2_MAJOR_VERSION ${MAPIT_MAJOR_VERSION})
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING MAPIT_MAJOR_VERSION
)

install(TARGETS ${PROJECT_NAME} EXPORT mapit_layertype_pointcloud2_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(DIRECTORY include/mapit DESTINATION include )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_layertype_pointcloud2ConfigVersion.cmake"
  VERSION 1
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT mapit_layertype_pointcloud2_targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/mapit_layertype_pointcloud2Targets.cmake"
  NAMESPACE mapit::
)
configure_file(cmake/mapit_layertype_pointcloud2Config.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_layertype_pointcloud2Config.cmake"
  COPYONLY
)

set(ConfigPackageLocation lib/cmake/mapit)
install(EXPORT mapit_layertype_pointcloud2_targets
  FILE
    mapit_layertype_pointcloud2Targets.cmake
  NAMESPACE
    mapit::
  DESTINATION
    ${ConfigPackageLocation}
)
install(
  FILES
    cmake/mapit_layertype_pointcloud2Config.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/mapit_layertype_pointcloud2ConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)
