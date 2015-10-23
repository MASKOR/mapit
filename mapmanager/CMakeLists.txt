project(upns_mapmanager)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

set(CMAKE_AUTOMOC ON)

find_package(Qt5Core ${UPNS_QTVERSION} REQUIRED)

use_leveldb()

link_directories(${EXTERNALS_DIR}/leveldb)
link_directories(${YAML_CPP_LIB_DIR})

#use_grpc()
use_protobuf()

include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${UPNS_INTERFACE_INCLUDE})
include_directories(${YAML_CPP_INCLUDE})

file(GLOB_RECURSE UPNS_MM_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h ${PROJECT_SOURCE_DIR}/include/*.h)

add_library(${PROJECT_NAME} ${SRC_LIST} ${UPNS_MM_SOURCES} )
target_link_libraries(${PROJECT_NAME} Qt5::Core upns_interface yaml-cpp ${EXTERNALS_DIR}/leveldb/libleveldb.so log4cplus)

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include/)
