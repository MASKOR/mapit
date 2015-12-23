project(mapman)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

set(CMAKE_AUTOMOC ON)

find_package(Qt5Core ${UPNS_QTVERSION} REQUIRED)
find_package( Boost 1.54 COMPONENTS program_options REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )

link_directories(${YAML_CPP_LIB_DIR})

use_protobuf()

include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${UPNS_INTERFACE_INCLUDE})
include_directories(${YAML_CPP_INCLUDE})

file(GLOB_RECURSE UPNS_MAPMAN_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

add_executable(${PROJECT_NAME} ${SRC_LIST} ${UPNS_MAPMAN_SOURCES} )
target_link_libraries(${PROJECT_NAME} Qt5::Core upns_mapmanager upns_interface yaml-cpp log4cplus ${Boost_PROGRAM_OPTIONS_LIBRARY})
