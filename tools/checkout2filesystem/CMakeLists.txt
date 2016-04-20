project(checkout2filesystem)
cmake_minimum_required(VERSION 2.8)

include(../cmake/custom_add_leveldb)

# add custom cmake scripts. i.e. FindProcessorCount...
set(CMAKE_MODULE_PATH
    ${PROJECT_SOURCE_DIR}/cmake
    ${CMAKE_MODULE_PATH}
)

use_protobuf()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)

INCLUDE_DIRECTORIES( UPNS_ROOT_DIR )

find_package(Qt5Core ${UPNS_QTVERSION} REQUIRED)

include(FindProcessorCount)
message(STATUS "used number of processor cores: " ${PROCESSOR_COUNT})

custom_set_vars_leveldb()
include_directories(${LEVELDB_INCLUDE_DIRECTORY})

include_directories(${YAML_CPP_INCLUDE})
include_directories(${PROJECT_SOURCE_DIR}/src/)

file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

add_executable(${PROJECT_NAME} ${UPNS_SOURCES})

if(WIN32)
  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES}  ${BOOST_LIBRARIES_ADDITION})
else(WIN32)
  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} uuid ${libzip} ${BOOST_LIBRARIES_ADDITION})
endif(WIN32)

target_link_libraries(${PROJECT_NAME} upns_mapmanager ${LEVELDB_LIBRARIES})
