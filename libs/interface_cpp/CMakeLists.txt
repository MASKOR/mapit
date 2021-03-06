################################################################################
#
# Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
#                2017 Marcus Meeßen	<marcus.meessen@alumni.fh-aachen.de>
#                2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

project(interface_cpp)
cmake_minimum_required(VERSION 2.8)

# interface_cpp contains headers with virtual interfaces, which can be implemented by different libraries.

file(GLOB_RECURSE INTERFACE_CPP_SOURCES include/*.h)

add_library(${PROJECT_NAME} INTERFACE) # Header only
target_include_directories(${PROJECT_NAME} INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                     $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} INTERFACE
                      interface)
#shows files in qtcreator
add_custom_target(${PROJECT_NAME}_NOP ALL ${CMAKE_COMMAND} -E echo "show files in qtcreator" SOURCES ${INTERFACE_CPP_SOURCES})

#install(TARGETS ${PROJECT_NAME} DESTINATION lib/${CMAKE_LIBRARY_ARCHITECTURE})
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${CMAKE_LIBRARY_ARCHITECTURE} )

include(GenerateExportHeader)

generate_export_header(${PROJECT_NAME})
#set_property(TARGET ${PROJECT_NAME} PROPERTY VERSION ${MAPIT_VERSION})
#set_property(TARGET ${PROJECT_NAME} PROPERTY SOVERSION ${MAPIT_VERSION})
set_property(TARGET ${PROJECT_NAME} PROPERTY
  INTERFACE_mapit_interface_cpp_MAJOR_VERSION ${MAPIT_MAJOR_VERSION})
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING MAPIT_MAJOR_VERSION
)

install(TARGETS ${PROJECT_NAME} EXPORT mapit_interface_cpp_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(DIRECTORY include/mapit DESTINATION include )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_interface_cppConfigVersion.cmake"
  VERSION 1
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT mapit_interface_cpp_targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/mapit_interface_cppTargets.cmake"
  NAMESPACE mapit::
)
configure_file(cmake/mapit_interface_cppConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_interface_cppConfig.cmake"
  COPYONLY
)

set(ConfigPackageLocation lib/cmake/mapit)
install(EXPORT mapit_interface_cpp_targets
  FILE
    mapit_interface_cppTargets.cmake
  NAMESPACE
    mapit::
  DESTINATION
    ${ConfigPackageLocation}
)
install(
  FILES
    cmake/mapit_interface_cppConfig.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/mapit_interface_cppConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)
