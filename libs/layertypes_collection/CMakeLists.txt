cmake_minimum_required(VERSION 2.8)

if(WIN32)
    # Dll treated as runtime
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/libs/layertypes_collection)
else(WIN32)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/libs/layertypes_collection)
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

set(WITH_PCL TRUE CACHE BOOL "Enable pcl Layers")
set(WITH_OPENVDB TRUE CACHE BOOL "Enable OpenVDB Layers")
set(WITH_OCTOMAP TRUE CACHE BOOL "Enable Octomap Layers")
set(WITH_TF TRUE CACHE BOOL "Enable Tf Layers, requiring Eigen2")

if(NOT WITH_PCL)
  list(REMOVE_ITEM SUBDIRS "pointcloud2")
endif(NOT WITH_PCL)

if(NOT WITH_OPENVDB)
  list(REMOVE_ITEM SUBDIRS "openvdb")
endif(NOT WITH_OPENVDB)

if(NOT WITH_OCTOMAP)
  list(REMOVE_ITEM SUBDIRS "octomap")
endif(NOT WITH_OCTOMAP)

if(NOT WITH_TF)
  list(REMOVE_ITEM SUBDIRS "tf")
endif(NOT WITH_TF)

FOREACH(subdir ${SUBDIRS})
    ADD_SUBDIRECTORY(${subdir})
ENDFOREACH()
