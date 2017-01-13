project(layertype_openvdb)

set(WITH_LAYERTYPE_OPENVDB TRUE CACHE BOOL "Enable OpenVDB Layers")
if(WITH_LAYERTYPE_OPENVDB)
  cmake_minimum_required(VERSION 2.8)
  aux_source_directory(. SRC_LIST)

  include_directories(${PROJECT_SOURCE_DIR}/src/)
  include_directories(${PROJECT_SOURCE_DIR}/../../mapmanager/include/)

  file(GLOB_RECURSE LAYERTYPE_OPENVDB_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

  add_library(${PROJECT_NAME} ${SRC_LIST} ${LAYERTYPE_OPENVDB_SOURCES} )
endif(WITH_LAYERTYPE_OPENVDB)
