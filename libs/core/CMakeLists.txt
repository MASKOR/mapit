################################################################################
#
# Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
#           2015-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
#                2016 Kai Krueckel	<kai.krueckel@alumni.fh-aachen.de>
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

project(core)
cmake_minimum_required(VERSION 2.8)

find_package(Boost REQUIRED COMPONENTS filesystem iostreams)

file(GLOB_RECURSE CORE_SOURCES src/*.cpp
                                  src/*.h
                                  src/serialization/file_system/*.cpp
                                  src/serialization/file_system/*.h
                                  include/*.h
                                  src/sha/*.cpp
                                  src/sha/*.h)

add_library(${PROJECT_NAME} SHARED ${CORE_SOURCES} )
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

#Note: Windows (and also linux in some cases?):
# PCL was linked statically against boost. Entitytype is linked statically against PCL.
# Entitytype has nothing to do with core. (linked dynamic and at runtime)
# Somehow boost dependency comes in here. I guess a static library wants it, thus the executable/dynamic-library must have it.
target_link_libraries(${PROJECT_NAME} interface interface_cpp ${Boost_LIBRARIES} ${BOOST_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY})

# tell the code which postfix to use for dynamic operator/layertype loading
add_definitions(-DDEBUG_POSTFIX="${CMAKE_DEBUG_POSTFIX}")

#TODO: maybe not needed
if(MAPIT_ENABLE_LOG4CPLUS)
  target_link_libraries(${PROJECT_NAME} ${LOG4CPLUS_LIBRARIES})
endif(MAPIT_ENABLE_LOG4CPLUS)

if(NOT WIN32)
    target_link_libraries(${PROJECT_NAME} dl)
endif(NOT WIN32)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
    $<INSTALL_INTERFACE:include>
  PRIVATE
    src
    ${Boost_INCLUDE_DIRS}
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME mapit_${PROJECT_NAME})

include(GenerateExportHeader)

generate_export_header(${PROJECT_NAME})
set_property(TARGET ${PROJECT_NAME} PROPERTY VERSION 1)
set_property(TARGET ${PROJECT_NAME} PROPERTY SOVERSION 1)
set_property(TARGET ${PROJECT_NAME} PROPERTY
  INTERFACE_mapit_core_MAJOR_VERSION 1)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING mapit_MAJOR_VERSION
)

install(TARGETS ${PROJECT_NAME} EXPORT mapit_core_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(DIRECTORY include/mapit DESTINATION include )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_coreConfigVersion.cmake"
  VERSION 1
  COMPATIBILITY AnyNewerVersion
)

# TODO: CMAKE_BINARY_DIR or CMAKE_CURRENT_BINARY_DIR?
export(EXPORT mapit_core_targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/mapit_coreTargets.cmake"
  NAMESPACE mapit::
)

configure_file(cmake/mapit_coreConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_coreConfig.cmake"
  @ONLY
)

set(ConfigPackageLocation lib/cmake/mapit)
install(EXPORT mapit_core_targets
  FILE
    mapit_coreTargets.cmake
  NAMESPACE
    mapit::
  DESTINATION
    ${ConfigPackageLocation}
)
install(
  FILES
    cmake/mapit_coreConfig.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/mapit_coreConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)
