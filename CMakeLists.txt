#cmake_minimum_required(VERSION 3.0) # for hunter
#set(HUNTER_SKIP_LOCK ON)
#set(HUNTER_ROOT ${PROJECT_SOURCE_DIR}/hunter)
#include("cmake/HunterGate.cmake")

#HunterGate(
#    URL "https://github.com/ruslo/hunter/archive/v0.12.24.tar.gz"
#    SHA1 "a25a7fa55c204a8126efd1f48593a662925b26da"
#)

# Note: DISABLE_PCL does not yet work with minimal custom headers of pcl.
# At least components io and common are needed
project(upns)
cmake_minimum_required(VERSION 2.8.12) # for target_include_directories
aux_source_directory(. SRC_LIST)

# c++11 for grpc dependencies
add_definitions(-std=c++11)

add_definitions(-DDEBUG_POSTFIX="${CMAKE_DEBUG_POSTFIX}")

set(UPNS_ROOT_DIR ${PROJECT_SOURCE_DIR})

add_definitions(-DDEBUG_POSTFIX="${CMAKE_DEBUG_POSTFIX}")
# add custom cmake scripts. i.e. FindProcessorCount...
set(CMAKE_MODULE_PATH
    ${PROJECT_SOURCE_DIR}/cmake
    ${CMAKE_MODULE_PATH}
)
include(externals)
include(FindLog4cplus)

#set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)

set(UPNS_QTVERSION)
include(FindProcessorCount)
message(STATUS "used number of processor cores: " ${PROCESSOR_COUNT})

set(CMAKE_C_FLAGS_DEBUG "-DUPNS_DEBUG")

# Note: Dependency management is yet a mess. Some projects are contained in externals folder.
# other projects are downloaded by cmake and installed
# other projects must be installed on host system
#find_package(Log4cplus REQUIRED)
#include_directories( ${LOG4CPLUS_INCLUDE_DIR} )


file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)


set(DataDir ${PROJECT_SOURCE_DIR}/data)
file(GLOB Datas ${DataDir}/*)
file(GLOB_RECURSE Datas_All ${DataDir}/*)

include_directories( ${CMAKE_BINARY_DIR} ) # for proto headers
include_directories( ${UPNS_ROOT_DIR} ) # for proto headers
include_directories(${LOG4CPLUS_INCLUDE_DIR})
include_directories(${BOOST_INCLUDE_DIR})

set(UPNS_INTERFACE_INCLUDE ${PROJECT_SOURCE_DIR}/libs/upns_interface/generated/)

add_subdirectory(libs/mapmanager)
add_subdirectory(libs/operator_modules)
add_subdirectory(test/unit_tests)
add_subdirectory(libs/upns_interface)
add_subdirectory(libs/interface_grpc)
add_subdirectory(libs/layertypes)
#add_subdirectory(commandlinetool_mapman)
#add_subdirectory(externals)
#add_subdirectory(externals/yaml-cpp)
if(ENABLE_VISUALIZATION)
  add_subdirectory(tools/visualization)
endif(ENABLE_VISUALIZATION)

#hunter_add_package(yaml-cpp)

include(ExternalProject) # ExternalProject_Add

set(YAML_CPP_INSTALL_DIR "${PROJECT_SOURCE_DIR}/externals/libs/yaml-cpp-0.5.2" CACHE PATH "yaml installation path with lib and include subdir")
set(YAML_CPP_INCLUDE ${YAML_CPP_INSTALL_DIR}/include)
set(YAML_CPP_LIB_DIR ${YAML_CPP_INSTALL_DIR}/lib)

#add_subdirectory(externals/yaml-cpp)

#ExternalProject_Add(
#    yaml-cpp
#    URL
#    "https://github.com/jbeder/yaml-cpp/archive/release-0.5.2.tar.gz"
#    CONFIGURE_COMMAND
#    ""
#    BUILD_COMMAND
#    mkdir -p build && cd build && cmake -DCMAKE_INSTALL_PREFIX:PATH=${YAML_CPP_INSTALL_DIR} .. && make
#    BUILD_IN_SOURCE
#    1
#    INSTALL_COMMAND
#    cd build && make install
#)

# make data visible in qtcreator
add_custom_target(resourceFiles ALL ${CMAKE_COMMAND} -E echo "add datas to qt project. nop" SOURCES ${Datas_All})

#target_link_libraries(${PROJECT_NAME} )

# make qml visible in qtcreator
file(GLOB_RECURSE UPNS_QMLFILES ${PROJECT_SOURCE_DIR}/*.qml ${PROJECT_SOURCE_DIR}/*/qmldir ${PROJECT_SOURCE_DIR}/*.properties)

add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add qml files to project" SOURCES ${UPNS_QMLFILES})
