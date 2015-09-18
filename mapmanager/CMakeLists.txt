project(upns_mapmanager)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

set(CMAKE_AUTOMOC ON)

find_package(Qt5Core ${UPNS_QTVERSION} REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/src/)

file(GLOB_RECURSE UPNS_MM_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

add_library(${PROJECT_NAME} ${SRC_LIST} ${UPNS_MM_SOURCES} )

target_link_libraries(${PROJECT_NAME} Qt5::Core )
