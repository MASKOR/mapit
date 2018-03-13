################################################################################
#
# Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
#                2015 Tobias Neumann	<t.neumann@fh-aachen.de>
#                2017 Marcus Meeßen	<marcus.meessen@alumni.fh-aachen.de>
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

cmake_minimum_required(VERSION 2.8)

if(WIN32)
    # Dll treated as runtime
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/libs)
else(WIN32)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/libs)
endif(WIN32)

SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "-rdynamic")

MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
        SET(dirlist ${dirlist} ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})

list(REMOVE_ITEM SUBDIRS "CMakeFiles") # this directory is created by cpack (TODO: validate)

if(NOT MAPIT_ENABLE_PCL)
  list(REMOVE_ITEM SUBDIRS "pointcloud2")
endif(NOT MAPIT_ENABLE_PCL)

if(NOT MAPIT_ENABLE_OPENVDB)
  list(REMOVE_ITEM SUBDIRS "openvdb")
endif(NOT MAPIT_ENABLE_OPENVDB)

if(NOT MAPIT_ENABLE_OCTOMAP)
  list(REMOVE_ITEM SUBDIRS "octomap")
endif(NOT MAPIT_ENABLE_OCTOMAP)

if(NOT MAPIT_ENABLE_TF)
  list(REMOVE_ITEM SUBDIRS "tf")
endif(NOT MAPIT_ENABLE_TF)

if(NOT WITH_LAS)
  list(REMOVE_ITEM SUBDIRS "las")
endif(NOT WITH_LAS)

FOREACH(subdir ${SUBDIRS})
    ADD_SUBDIRECTORY(${subdir})
ENDFOREACH()
