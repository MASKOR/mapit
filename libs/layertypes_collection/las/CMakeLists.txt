################################################################################
#
# Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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

project(layertype_las)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

find_package(Boost COMPONENTS program_options thread system iostreams filesystem REQUIRED)

find_package(mapit)

if(WIN32)
  include_directories(${PROJECT_SOURCE_DIR}/../../../externals/windows/include/)
endif(WIN32)

add_definitions(-DPROJECT_NAME="${PROJECT_NAME}")

file(GLOB_RECURSE LAYERTYPE_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/src/*.h
                                                ${PROJECT_SOURCE_DIR}/libLAS/include/*.hpp
                                                ${PROJECT_SOURCE_DIR}/libLAS/include/*.h
                                                ${PROJECT_SOURCE_DIR}/libLAS/src/*.hpp
                                                ${PROJECT_SOURCE_DIR}/libLAS/src/*.h
                                                ${PROJECT_SOURCE_DIR}/libLAS/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/include/*.h )

set(HAVE_GDAL FALSE CACHE BOOL "Enable GDAL for LibLAS")
set(HAVE_LIBGEOTIFF FALSE CACHE BOOL "Enable libGeoTIFF for LibLAS")
set(HAVE_LASZIP TRUE CACHE BOOL "Enable Laszip for LibLAS")

if(NOT HAVE_GDAL)
  file(GLOB_RECURSE GT_SRC ${PROJECT_SOURCE_DIR}/libLAS/src/gt_*.cpp
                           ${PROJECT_SOURCE_DIR}/libLAS/src/gt_*.h)
  list(REMOVE_ITEM LAYERTYPE_SOURCES ${GT_SRC})
endif(NOT HAVE_GDAL)

if(NOT HAVE_LIBGEOTIFF)
  file(GLOB_RECURSE GEO_SRC ${PROJECT_SOURCE_DIR}/libLAS/src/tifvsi.cpp)
  list(REMOVE_ITEM LAYERTYPE_SOURCES ${GEO_SRC})
endif(NOT HAVE_LIBGEOTIFF)

if(HAVE_LASZIP)
  file(GLOB_RECURSE LASZIP_SRC ${PROJECT_SOURCE_DIR}/LASzip/src/* ${PROJECT_SOURCE_DIR}/LASzip/include/*)
endif(HAVE_LASZIP)

add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${LAYERTYPE_SOURCES} ${LASZIP_SRC})
target_link_libraries( ${PROJECT_NAME} ${mapit_interface_cpp} ${BOOST_LIBRARIES} ${Boost_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                   $<INSTALL_INTERFACE:include>
                                                   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libLAS/include>
                                                   $<INSTALL_INTERFACE:libLAS/include>
                                           PRIVATE ${PROJECT_SOURCE_DIR}/src/
                                                   ${Boost_INCLUDE_DIRS})

target_compile_definitions(${PROJECT_NAME} PRIVATE PROJECT_NAME="${PROJECT_NAME}"
                                           PUBLIC  LASEntitydata_TYPENAME="${PROJECT_NAME}"
                                                   WITH_LAS=1)

if(HAVE_LASZIP)
   target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/LASzip/src/
                                                      ${PROJECT_SOURCE_DIR}/LASzip/include/
                                                      ${PROJECT_SOURCE_DIR}/LASzip/include/laszip)
   target_compile_definitions(${PROJECT_NAME} PUBLIC HAVE_LASZIP)
endif(HAVE_LASZIP)

if(UNIX)
    # tell CMake Prefix is "lib" otherwise it can not be queried
    set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib")
endif(UNIX)
get_target_property(OLD_PREFIX ${PROJECT_NAME} PREFIX)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX ${OLD_PREFIX}${UPNS_INSTALL_LAYERTYPES})

#install(TARGETS ${PROJECT_NAME} DESTINATION lib)
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${CMAKE_LIBRARY_ARCHITECTURE})

include(GenerateExportHeader)

generate_export_header(${PROJECT_NAME})
#set_property(TARGET ${PROJECT_NAME} PROPERTY VERSION ${MAPIT_VERSION})
#set_property(TARGET ${PROJECT_NAME} PROPERTY SOVERSION ${MAPIT_VERSION})
set_property(TARGET ${PROJECT_NAME} PROPERTY
  INTERFACE_mapit_layertype_las_MAJOR_VERSION ${MAPIT_MAJOR_VERSION})
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING MAPIT_MAJOR_VERSION
)

install(TARGETS ${PROJECT_NAME} EXPORT mapit_layertype_las_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(DIRECTORY include/upns DESTINATION include )
#install(DIRECTORY include/mapit DESTINATION include )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_layertype_lasConfigVersion.cmake"
  VERSION 1
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT mapit_layertype_las_targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/mapit_layertype_lasTargets.cmake"
  NAMESPACE mapit::
)
configure_file(cmake/mapit_layertype_lasConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_layertype_lasConfig.cmake"
  COPYONLY
)

set(ConfigPackageLocation lib/cmake/mapit)
install(EXPORT mapit_layertype_las_targets
  FILE
    layertype_lasTargets.cmake
  NAMESPACE
    mapit::
  DESTINATION
    ${ConfigPackageLocation}
)
install(
  FILES
    cmake/mapit_layertype_lasConfig.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/mapit_layertype_lasConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)
