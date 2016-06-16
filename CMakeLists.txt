
# TODO: Cleanup Cmake
# 1) Put all frameworks in root cmake once. Make them optional. PCL, Boost (some components may be required for mapmanager), OpenVdb, Assimp
# 2) compile some operators/layertypes only if these frameworks are found
# 3) Make pcl cmake silent? Remove its clumsy output
# 4) Output which frameworks were found and wich projects will/will not be built.
# 5) Remove additional path to configure boost, ...! Maybe add one path and rely on librarynames.
# 6) Make compilation type independent (Remove Cmake_Build_Type == Debug queries)

project(upns)
cmake_minimum_required(VERSION 2.8.12) # for target_include_directories
aux_source_directory(. SRC_LIST)

set (CMAKE_CXX_STANDARD 11)

if(NOT WIN32)
# c++11 for grpc dependencies
add_definitions(-std=c++11)
endif(NOT WIN32)

add_definitions(-DDEBUG_POSTFIX="${CMAKE_DEBUG_POSTFIX}")
if(${VRMODE})
add_definitions(-DVRMODE="${VRMODE}")
endif(${VRMODE})

set(UPNS_ROOT_DIR ${PROJECT_SOURCE_DIR})

# add custom cmake scripts. i.e. FindProcessorCount...
set(CMAKE_MODULE_PATH
    ${PROJECT_SOURCE_DIR}/cmake
    ${CMAKE_MODULE_PATH}
)
include(externals)
include(FindLog4cplus)

# These scripts will publish variables to be set (e.g. using cmake-gui).
# They also publish macros (custom_target_use_XXX(${TARGET}) ) wich add header and libraries to target.
# If the framework was not found, readable output is given about what to do.
include(custom_add_boost)
include(custom_add_log4cplus)
include(custom_add_protobuf)
include(custom_add_yamlcpp)
include(custom_add_leveldb)
include(custom_add_pcl)
include(custom_add_assimp)
include(custom_add_openvdb)
include(process_protobuf)

custom_set_vars_boost()
find_package(Boost COMPONENTS filesystem system date_time serialization)

set(CMAKE_AUTOMOC ON)

set(UPNS_QTVERSION)
include(FindProcessorCount)
message(STATUS "used number of processor cores: " ${PROCESSOR_COUNT})

string( TOLOWER "${CMAKE_BUILD_TYPE}" LOWER_CMAKE_BUILD_TYPE )
if (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
    add_definitions("-DUPNS_DEBUG")
endif()
# Note: Dependency management is yet a mess. Some projects are contained in externals folder.
# other projects are downloaded by cmake and installed
# other projects must be installed on host system
#find_package(Log4cplus REQUIRED)
#include_directories( ${LOG4CPLUS_INCLUDE_DIR} )


file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)


set(DataDir ${PROJECT_SOURCE_DIR}/data)
file(GLOB_RECURSE Datas_All ${DataDir}/*)

set(ResourceDir ${PROJECT_SOURCE_DIR}/resources)
file(GLOB_RECURSE Resources_All ${ResourceDir}/*)

include_directories( ${CMAKE_BINARY_DIR} ) # for proto headers
include_directories( ${UPNS_ROOT_DIR} ) # for proto headers
include_directories(${LOG4CPLUS_INCLUDE_DIR})
include_directories(${UPNS_ROOT_DIR}/externals/include)
#include_directories(${BOOST_INCLUDE_DIR})

set(UPNS_INTERFACE_INCLUDE ${PROJECT_SOURCE_DIR}/libs/upns_interface/generated/)

set(ENABLE_VISUALIZATION false CACHE BOOL "Generate visualization projects. This may introduce hard to statisfy dependecies.")

add_subdirectory(libs/mapmanager)
add_subdirectory(libs/operator_modules_collection)
add_subdirectory(test/unit_tests)
add_subdirectory(libs/upns_interface)
add_subdirectory(libs/interface_grpc)
add_subdirectory(libs/layertypes_collection)
#add_subdirectory(commandlinetool_mapman)
#add_subdirectory(externals)
#add_subdirectory(externals/yaml-cpp)
add_subdirectory(tools/databasedump)
add_subdirectory(tools/checkout2filesystem)
add_subdirectory(tools/execute_operator)
add_subdirectory(tools/checkout_create)
add_subdirectory(tools/log)
add_subdirectory(tools/commit)
add_subdirectory(tools/scripts)
if(ENABLE_VISUALIZATION)
  #add_subdirectory(tools/visualization)
  add_subdirectory(tools/visualization_renderview)
  add_subdirectory(tools/visualization_standalone)
  add_subdirectory(tools/Qt3DPointcloudRenderer)
endif(ENABLE_VISUALIZATION)

#hunter_add_package(yaml-cpp)

include(ExternalProject) # ExternalProject_Add

set(YAML_CPP_INSTALL_DIR "${PROJECT_SOURCE_DIR}/externals/yaml-cpp/build" CACHE PATH "yaml installation path with lib and include subdir")
set(YAML_CPP_INCLUDE "${YAML_CPP_INSTALL_DIR}/include" CACHE PATH "yaml include dir")
set(YAML_CPP_LIB_DIR "${YAML_CPP_INSTALL_DIR}" CACHE PATH "yaml lib dir")
set(YAML_CPP_LIBRARIES "${YAML_CPP_LIB_DIR}/libyaml-cpp.a" CACHE FILEPATH "yaml lib")

#set(BOOST_ADDITIONAL_LINK_DIRECTORY "somewhere" CACHE PATH "")
#link_directories(${BOOST_ADDITIONAL_LINK_DIRECTORY})
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

# Note: QtCreator crashes when text is searched and it starts to search in Gigabytes of Pointclouds
# make data and resources visible in qtcreator
#add_custom_target(dataFiles ALL ${CMAKE_COMMAND} -E echo "add datas to qt project. nop" SOURCES ${Datas_All})
add_custom_target(resourceFiles ALL ${CMAKE_COMMAND} -E echo "add resources to qt project. nop" SOURCES ${Resources_All})

#target_link_libraries(${PROJECT_NAME} )

# make qml visible in qtcreator
file(GLOB_RECURSE UPNS_QMLFILES ${PROJECT_SOURCE_DIR}/*.qml ${PROJECT_SOURCE_DIR}/*/qmldir ${PROJECT_SOURCE_DIR}/*.properties)

add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add qml files to project" SOURCES ${UPNS_QMLFILES})
