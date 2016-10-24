
# TODO: Cleanup Cmake
# 1) Put all frameworks in root cmake once. Make them optional. PCL, Boost (some components may be required for mapmanager), OpenVdb, Assimp
# 2) compile some operators/layertypes only if these frameworks are found
# 3) Make pcl cmake silent? Remove its clumsy output
# 4) Output which frameworks were found and wich projects will/will not be built.
# 5) Remove additional path to configure boost, ...! Maybe add one path and rely on librarynames.
# 6) Make compilation type independent (Remove Cmake_Build_Type == Debug queries)

project(upns)
cmake_minimum_required(VERSION 3.0.2) # for target_include_directories
aux_source_directory(. SRC_LIST)

#set (CMAKE_CXX_STANDARD 14)

if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
        message(WARNING "WARNING: In-Place Build not recommended.")
endif()

if(NOT WIN32)
# c++11 for grpc dependencies
#add_definitions(-std=c++11)
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

include(FindLog4cplus)

if(LOG4CPLUS_FOUND)
    add_definitions(-DLOG4CPLUS_FOUND)
endif(LOG4CPLUS_FOUND)


if(WIN32)
    set(BOOST_ROOT ${BOOST_INCLUDE_DIR} CACHE PATH "Hint for boost libraries")
    if(MSVC10)
      SET ( BOOST_LIBRARYDIR "${BOOST_ROOT}/lib64-msvc-10.0" )
    endif(MSVC10)
    if(MSVC12)
      SET ( BOOST_LIBRARYDIR "${BOOST_ROOT}/lib64-msvc-12.0" )
    endif(MSVC12)
    if(MSVC14)
      SET ( BOOST_LIBRARYDIR "${BOOST_ROOT}/lib64-msvc-14.0" )
    endif(MSVC14)
    find_library (Boost_CHRONO_LIBRARY_DEBUG libboost_chrono-vc120-mt-gd-1_56.lib PATHS ${BOOST_LIBRARYDIR}/ CACHE FILEPATH "boostlib" FORCE)
    find_library (Boost_CHRONO_LIBRARY_RELEASE libboost_chrono-vc120-mt-gd-1_56.lib PATHS ${BOOST_LIBRARYDIR}/ CACHE FILEPATH "boostlib" FORCE)
    find_library (Boost_DATE_TIME_LIBRARY_DEBUG libboost_datetime-vc120-mt-gd-1_56.lib PATHS ${BOOST_LIBRARYDIR}/ CACHE FILEPATH "boostlib" FORCE)
    find_library (Boost_DATE_TIME_LIBRARY_RELEASE libboost_datetime-vc120-mt-gd-1_56.lib PATHS ${BOOST_LIBRARYDIR}/ CACHE FILEPATH "boostlib" FORCE)
    find_library (Boost_FILESYSTEM_LIBRARY_DEBUG libboost_filesystem-vc120-mt-gd-1_56.lib PATHS ${BOOST_LIBRARYDIR}/ CACHE FILEPATH "boostlib" FORCE)
    find_library (Boost_FILESYSTEM_LIBRARY_RELEASE libboost_filesystem-vc120-mt-gd-1_56.lib PATHS ${BOOST_LIBRARYDIR}/ CACHE FILEPATH "boostlib" FORCE)
    find_library (Boost_IOSTREAMS_LIBRARY_DEBUG libboost_iostreams-vc120-mt-gd-1_56.lib PATHS ${BOOST_LIBRARYDIR}/ CACHE FILEPATH "boostlib" FORCE)
    find_library (Boost_IOSTREAMS_LIBRARY_RELEASE libboost_iostreams-vc120-mt-gd-1_56.lib PATHS ${BOOST_LIBRARYDIR}/ CACHE FILEPATH "boostlib" FORCE)
    find_library (Boost_SERIALIZATION_LIBRARY_DEBUG libboost_serialization-vc120-mt-1_56.lib PATHS ${BOOST_LIBRARYDIR}/ CACHE FILEPATH "boostlib" FORCE)
    find_library (Boost_SERIALIZATION_LIBRARY_RELEASE libboost_serialization-vc120-mt-gd-1_56.lib PATHS ${BOOST_LIBRARYDIR}/ CACHE FILEPATH "boostlib" FORCE)
    find_library (Boost_SYSTEM_LIBRARY_DEBUG libboost_system-vc120-mt-gd-1_56.lib PATHS ${BOOST_LIBRARYDIR}/ CACHE FILEPATH "boostlib" FORCE)
    find_library (Boost_SYSTEM_LIBRARY_RELEASE libboost_system-vc120-mt-gd-1_56.lib PATHS ${BOOST_LIBRARYDIR}/ CACHE FILEPATH "boostlib" FORCE)
    find_library (Boost_THREAD_LIBRARY_DEBUG libboost_thread-vc120-mt-gd-1_56.lib PATHS ${BOOST_LIBRARYDIR}/ CACHE FILEPATH "boostlib" FORCE)
    find_library (Boost_THREAD_LIBRARY_RELEASE libboost_thread-vc120-mt-gd-1_56.lib PATHS ${BOOST_LIBRARYDIR}/ CACHE FILEPATH "boostlib" FORCE)
endif(WIN32)

find_package(Boost COMPONENTS filesystem system date_time serialization)

link_directories(${Boost_LIBRARY_DIRS})
set(CMAKE_AUTOMOC ON)

set(UPNS_QTVERSION)
include(FindProcessorCount)
message(STATUS "used number of processor cores: " ${PROCESSOR_COUNT})

string( TOLOWER "${CMAKE_BUILD_TYPE}" LOWER_CMAKE_BUILD_TYPE )
if (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
    add_definitions("-DUPNS_DEBUG")
endif()

file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

set(DataDir ${PROJECT_SOURCE_DIR}/data)
file(GLOB_RECURSE Datas_All ${DataDir}/*)

set(ResourceDir ${PROJECT_SOURCE_DIR}/resources)
file(GLOB_RECURSE Resources_All ${ResourceDir}/*)

include_directories( ${CMAKE_BINARY_DIR} ) # for proto headers
include_directories( ${UPNS_ROOT_DIR} ) # for proto headers
include_directories(${LOG4CPLUS_INCLUDE_DIR})
#include_directories(${UPNS_ROOT_DIR}/externals/include) # for sha initially

set(UPNS_INTERFACE_INCLUDE ${CMAKE_CURRENT_BINARY_DIR}/libs/upns_interface/)

set(ENABLE_VISUALIZATION false CACHE BOOL "Generate visualization projects. This may introduce hard to statisfy dependecies.")

add_subdirectory(libs/mapmanager)
add_subdirectory(libs/operator_modules_collection)
add_subdirectory(test/unit_tests)
add_subdirectory(libs/upns_interface)
add_subdirectory(libs/upns_interface_cpp)
add_subdirectory(libs/interface_grpc)
add_subdirectory(libs/layertypes_collection)
add_subdirectory(libs/networking_node)
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
add_subdirectory(tools/server)

set(UPNS_ENABLE_VISUALIZATION ON CACHE BOOL "Generate Visualization tools")

if(UPNS_ENABLE_VISUALIZATION)
  #add_subdirectory(tools/visualization)
  add_subdirectory(tools/visualization_renderview)
  add_subdirectory(tools/visualization_standalone)
  add_subdirectory(tools/Qt3DPointcloudRenderer)
endif(UPNS_ENABLE_VISUALIZATION)

if(WIN32)
    set(YAML_CPP_INSTALL_DIR "${PROJECT_SOURCE_DIR}/externals/yaml-cpp/build" CACHE PATH "yaml installation path with lib and include subdir")
    set(YAML_CPP_INCLUDE "${PROJECT_SOURCE_DIR}/externals/windows/include" CACHE PATH "yaml include dir")
    set(YAML_CPP_LIB_DIR "${PROJECT_SOURCE_DIR}/externals/windows/lib-msvc2013-md-64" CACHE PATH "yaml lib dir")
    find_library(YAML_CPP_LIBRARY_RELEASE libyaml-cppmd.lib PATHS ${YAML_CPP_LIB_DIR} DOC "yaml lib")
    find_library(YAML_CPP_LIBRARY_DEBUG libyaml-cppmdd.lib PATHS ${YAML_CPP_LIB_DIR} DOC "yaml lib")
else(WIN32)
    set(YAML_CPP_INSTALL_DIR "${PROJECT_SOURCE_DIR}/externals/yaml-cpp/build" CACHE PATH "yaml installation path with lib and include subdir")
    set(YAML_CPP_INCLUDE "${YAML_CPP_INSTALL_DIR}/include" CACHE PATH "yaml include dir")
    set(YAML_CPP_LIB_DIR "${YAML_CPP_INSTALL_DIR}" CACHE PATH "yaml lib dir")
    find_library(YAML_CPP_LIBRARY_RELEASE libyaml-cpp.a PATHS ${YAML_CPP_LIB_DIR} DOC "yaml lib")
    find_library(YAML_CPP_LIBRARY_DEBUG libyaml-cpp.a PATHS ${YAML_CPP_LIB_DIR} DOC "yaml lib")
    #set(YAML_CPP_LIBRARIES debug ${YAML_LIB_DEBUG} optimized ${YAML_LIB_RELEASE})
endif(WIN32)
if(WIN32)
    set(YAML_CPP_LIBRARIES debug "${YAML_CPP_LIBRARY_DEBUG}" optimized "${YAML_CPP_LIBRARY_RELEASE}")
else(WIN32)
    set(YAML_CPP_LIBRARIES "${YAML_CPP_LIBRARY_DEBUG}" "${YAML_CPP_LIBRARY_RELEASE}")
endif(WIN32)
#mark_as_advanced(VAR ${YAML_CPP_LIBRARIES})

# Note: QtCreator crashes when text is searched and it starts to search in Gigabytes of Pointclouds
# make data and resources visible in qtcreator
#add_custom_target(dataFiles ALL ${CMAKE_COMMAND} -E echo "add datas to qt project. nop" SOURCES ${Datas_All})
add_custom_target(resourceFiles ALL ${CMAKE_COMMAND} -E echo "add resources to qt project. nop" SOURCES ${Resources_All})

# make qml visible in qtcreator
file(GLOB_RECURSE UPNS_QMLFILES ${PROJECT_SOURCE_DIR}/*.qml ${PROJECT_SOURCE_DIR}/*/qmldir ${PROJECT_SOURCE_DIR}/*.properties)

add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add qml files to project" SOURCES ${UPNS_QMLFILES})
