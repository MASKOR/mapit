################################################################################
#
# Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
#           2015-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

# For libraries with current json implementation
find_package(Qt5Core REQUIRED)


set(WITH_OPERATORS TRUE CACHE BOOL "Enable operators")
if(WITH_OPERATORS)
  SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})

  #list(REMOVE_ITEM SUBDIRS "common")
  list(REMOVE_ITEM SUBDIRS "CMakeFiles") # this directory is created by cpack (TODO: validate)
  if(NOT MAPIT_ENABLE_OPENVDB)
    list(REMOVE_ITEM SUBDIRS "levelset_to_mesh")
    list(REMOVE_ITEM SUBDIRS "ovdb_smooth")
    list(REMOVE_ITEM SUBDIRS "surfrecon_openvdb")
  endif(NOT MAPIT_ENABLE_OPENVDB)
  if(NOT MAPIT_ENABLE_PCL)
    list(REMOVE_ITEM SUBDIRS "centroid_to_origin")
    list(REMOVE_ITEM SUBDIRS "load_pointcloud")
    list(REMOVE_ITEM SUBDIRS "normalestimation")
    list(REMOVE_ITEM SUBDIRS "surfrecon_openvdb")
    list(REMOVE_ITEM SUBDIRS "filter_voxelgrid")
    list(REMOVE_ITEM SUBDIRS "moving_least_squares")
    list(REMOVE_ITEM SUBDIRS "fuse_pointclouds")
    list(REMOVE_ITEM SUBDIRS "pointclouds2octomap")
  endif(NOT MAPIT_ENABLE_PCL)
  if(NOT MAPIT_ENABLE_OCTOMAP)
    list(REMOVE_ITEM SUBDIRS "pointclouds2octomap")
    list(REMOVE_ITEM SUBDIRS "octomap2occupancygrid")
  endif()
  if(NOT WITH_LAS)
    list(REMOVE_ITEM SUBDIRS "load_las_from_csv")
    list(REMOVE_ITEM SUBDIRS "load_las")
    list(REMOVE_ITEM SUBDIRS "pcd2las")
    list(REMOVE_ITEM SUBDIRS "las2pcd")
  endif(NOT WITH_LAS)
  if(TRUE)
    list(REMOVE_ITEM SUBDIRS "surface_reconstruction")
  endif()

  if(NOT Qt5Core_FOUND)
      list(REMOVE_ITEM SUBDIRS "centroid_to_origin")
      list(REMOVE_ITEM SUBDIRS "filter_approximate_voxel_grid")
  endif(NOT Qt5Core_FOUND)

  FOREACH(subdir ${SUBDIRS})
    ADD_SUBDIRECTORY(${subdir})
  ENDFOREACH()
endif(WITH_OPERATORS)
