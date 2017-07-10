## New Cmake:
# no upns_ prefix anymore
# project can be used
## 1) within one single CMake structure,
## 2) installed
## 3) build directory (this has to be added to CMAKE_PREFIX_PATH for importing project)
# use ${mapit_*} for compatibility of project structure
# use mapit:: namespace only for exported/imported targets
# use no prefix internally only
# find_package(mapit) should work and make available ${mapit_*}, pointing to internal target or imported targets
# operators/layertypes must call find_package(mapit) and depend on mapit::* targets
# layertypes must export their targets. E.g. mapit_layertype_pointcloud2
# when depending on a layertype, find_package(layertype) must be called first
# the single mapitConfig.cmake configures all modules. This is a bit monolithic at the moment, but works.
# For the future, the configuration (loggin enabled, pcl enabled, ...) should be exported when installed/built too.
# Preprocessor defines for included headers should not differ when building mapit and using mapit.

# TODO: Cleanup Cmake
# 1) compile some operators/layertypes only if these frameworks are found
# 2) Make pcl cmake silent? Remove its clumsy output, also boost
# 3) Output which frameworks were found and wich projects will/will not be built.
# 4) Remove additional path to configure boost, ...! Maybe add one path and rely on librarynames. Pcl seems to break findBoost.
# 5) Make compilation type independent (Remove Cmake_Build_Type == Debug queries)

# TODOS:
# create Config.cmakes for all layertypes

project(upns)
cmake_minimum_required(VERSION 3.0.2) # for target_include_directories
aux_source_directory(. SRC_LIST)

set (CMAKE_CXX_STANDARD 11)

set(MAPIT_MAJOR_VERSION 0)
set(MAPIT_MINOR_VERSION 1)
set(MAPIT_PATCH_VERSION 0)
set(MAPIT_VERSION
  ${MAPIT_MAJOR_VERSION}.${MAPIT_MINOR_VERSION}.${MAPIT_PATCH_VERSION})

# This is used in the find script to determine, if targets non-imported projects can be used
#set(GLOBAL MAPIT_CMAKE_TARGETS_AVAILABLE TRUE)
set(MAPIT_CMAKE_TARGETS_AVAILABLE ON CACHE INTERNAL "Do not use, always true")

set(UPNS_INSTALL_INCLUDE "upns")
set(MAPIT_INSTALL_INCLUDE "mapit") # TODO: replace UPNS_* with MAPIT
# include directory as subfolder. final form: #include <${UPNS_INSTALL_INCLUDE}/${UPNS_INSTALL_INCLUDE_LAYERTYPES}/*.h>
set(UPNS_INSTALL_INCLUDE_LAYERTYPES "layertypes")
set(UPNS_INSTALL_LAYERTYPES "upns_layertypes_") # prefix for shared libraries
set(UPNS_INSTALL_OPERATORS "upns_operators_") # prefix for shared libraries
add_definitions(-DUPNS_INSTALL_LAYERTYPES="${UPNS_INSTALL_LAYERTYPES}")
add_definitions(-DUPNS_INSTALL_OPERATORS="${UPNS_INSTALL_OPERATORS}")

if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
        message(WARNING "WARNING: In-Place Build not recommended. If you accidently commit and push wrong files to the repository, you owe a cup of coffee to your teammembers.")
        return(1) # stop messing up source directory if this happens accidently
endif()

if(NOT WIN32)
# c++11 for grpc dependencies
#add_definitions(-std=c++11)
endif(NOT WIN32)

add_definitions(-DDEBUG_POSTFIX="${CMAKE_DEBUG_POSTFIX}")
if(VRMODE)
add_definitions(-DVRMODE="${VRMODE}")
endif(VRMODE)

set(UPNS_ROOT_DIR ${PROJECT_SOURCE_DIR})

# add custom cmake scripts. i.e. FindProcessorCount...
set(CMAKE_MODULE_PATH
    ${PROJECT_SOURCE_DIR}/cmake
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_MODULE_PATH}
)

# to make find_package(mapit) in build dir work
set(CMAKE_PREFIX_PATH
    ${PROJECT_SOURCE_DIR}/cmake
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_PREFIX_PATH}
)

##### let layertypes and operators know what libraries are available #####
# Note: This could be part of the layertypes cmake because all operators will depend on the layertype
# however at this point cmake already created the target for the operator and it might be hard to
# remove it again. Which layertypes/operators are build is decided in a single space
# (layertypes_collection and operator_modules_collection)
set(WITH_PCL TRUE CACHE BOOL "Enable pcl Layers")
set(WITH_OPENVDB TRUE CACHE BOOL "Enable OpenVDB Layers")
set(WITH_OCTOMAP TRUE CACHE BOOL "Enable Octomap Layers")
set(WITH_TF TRUE CACHE BOOL "Enable Tf Layers, requiring Eigen2")

set(WITH_LOG4CPLUS FALSE CACHE BOOL "Enable logging with log4cplus. Uses standard c out as an alternative")

if(WITH_LOG4CPLUS)
  include(FindLog4cplus)
  if(LOG4CPLUS_FOUND)
    add_definitions(-DLOG4CPLUS_FOUND)
    add_definitions(-DWITH_LOG4CPLUS)
  endif(LOG4CPLUS_FOUND)
endif(WITH_LOG4CPLUS)


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

include(CPack)

string( TOLOWER "${CMAKE_BUILD_TYPE}" LOWER_CMAKE_BUILD_TYPE )
if (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
    add_definitions("-DUPNS_DEBUG")
endif()

file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

set(DataDir ${PROJECT_SOURCE_DIR}/data)
file(GLOB_RECURSE Datas_All ${DataDir}/*)

set(ResourceDir ${PROJECT_SOURCE_DIR}/resources)
file(GLOB_RECURSE Resources_All ${ResourceDir}/*)

#include_directories( ${CMAKE_BINARY_DIR} ) # for proto headers
#include_directories( ${UPNS_ROOT_DIR} ) # for proto headers
if(WITH_LOG4CPLUS)
  include_directories(${LOG4CPLUS_INCLUDE_DIR})
endif(WITH_LOG4CPLUS)
#include_directories(${UPNS_ROOT_DIR}/externals/include) # for sha initially

#set(UPNS_INTERFACE_INCLUDE ${CMAKE_CURRENT_BINARY_DIR}/libs/upns_interface/)

add_subdirectory(libs/core)
add_subdirectory(libs/operator_modules_collection)
add_subdirectory(libs/upns_interface)
add_subdirectory(libs/upns_interface_cpp)
add_subdirectory(libs/layertypes_collection)
add_subdirectory(libs/networking_node)
#add_subdirectory(libs/multi_repository)
add_subdirectory(libs/standard_repository_factory)
#add_subdirectory(externals)
#add_subdirectory(externals/yaml-cpp)
#add_subdirectory(tools/databasedump)
add_subdirectory(tools/checkout2filesystem)
add_subdirectory(tools/execute_operator)
add_subdirectory(tools/checkout_create)
add_subdirectory(tools/log)
add_subdirectory(tools/commit)
add_subdirectory(tools/scripts)
add_subdirectory(tools/mapitd)
add_subdirectory(tools/commandlinetool)

set(UPNS_ENABLE_TESTS ON CACHE BOOL "Generate tests (need Qt)")
if(UPNS_ENABLE_TESTS)
  add_subdirectory(test/unit_tests)
endif()

# json11 is a static library that is meant to be used in a shared library.
# Thus the cmake target itself is added and slightly reconfigured here
if(WIN32)
  add_subdirectory(externals/json11)
  set_property(TARGET json11 PROPERTY POSITION_INDEPENDENT_CODE 1)
endif(WIN32)

set(UPNS_ENABLE_VISUALIZATION ON CACHE BOOL "Generate Visualization tools")

if(UPNS_ENABLE_VISUALIZATION)
  add_subdirectory(tools/visualization)
  add_subdirectory(tools/visualization_standalone)
  add_subdirectory(tools/Qt3DPointcloudRenderer)
endif(UPNS_ENABLE_VISUALIZATION)


# a config file which can be included with find_package(mapit) and published targets mapit::core, mapit::interface_cpp, ...
configure_file(cmake/mapitConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/mapitConfig.cmake"
  @ONLY
)

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)


#if(WIN32)
#    set(YAML_CPP_INSTALL_DIR "${PROJECT_SOURCE_DIR}/externals/yaml-cpp/build" CACHE PATH "yaml installation path with lib and include subdir")
#    set(YAML_CPP_INCLUDE "${PROJECT_SOURCE_DIR}/externals/windows/include" CACHE PATH "yaml include dir")
#    set(YAML_CPP_LIB_DIR "${PROJECT_SOURCE_DIR}/externals/windows/lib-msvc2013-md-64" CACHE PATH "yaml lib dir")
#    find_library(YAML_CPP_LIBRARY_RELEASE libyaml-cppmd.lib PATHS ${YAML_CPP_LIB_DIR} DOC "yaml lib")
#    find_library(YAML_CPP_LIBRARY_DEBUG libyaml-cppmdd.lib PATHS ${YAML_CPP_LIB_DIR} DOC "yaml lib")
#else(WIN32)
#    set(YAML_CPP_INSTALL_DIR "${PROJECT_SOURCE_DIR}/externals/yaml-cpp/build" CACHE PATH "yaml installation path with lib and include subdir")
#    set(YAML_CPP_INCLUDE "${YAML_CPP_INSTALL_DIR}/include" CACHE PATH "yaml include dir")
#    set(YAML_CPP_LIB_DIR "${YAML_CPP_INSTALL_DIR}" CACHE PATH "yaml lib dir")
#    find_library(YAML_CPP_LIBRARY_RELEASE libyaml-cpp.a PATHS ${YAML_CPP_LIB_DIR} DOC "yaml lib")
#    find_library(YAML_CPP_LIBRARY_DEBUG libyaml-cpp.a PATHS ${YAML_CPP_LIB_DIR} DOC "yaml lib")
#    #set(YAML_CPP_LIBRARIES debug ${YAML_LIB_DEBUG} optimized ${YAML_LIB_RELEASE})
#endif(WIN32)
#if(WIN32)
#    set(YAML_CPP_LIBRARIES debug "${YAML_CPP_LIBRARY_DEBUG}" optimized "${YAML_CPP_LIBRARY_RELEASE}")
#else(WIN32)
#    set(YAML_CPP_LIBRARIES "${YAML_CPP_LIBRARY_DEBUG}" "${YAML_CPP_LIBRARY_RELEASE}")
#endif(WIN32)
##mark_as_advanced(VAR ${YAML_CPP_LIBRARIES})

# Note: QtCreator crashes when text is searched and it starts to search in Gigabytes of Pointclouds
# make data and resources visible in qtcreator
#add_custom_target(dataFiles ALL ${CMAKE_COMMAND} -E echo "add datas to qt project. nop" SOURCES ${Datas_All})
#add_custom_target(resourceFiles ALL ${CMAKE_COMMAND} -E echo "add resources to qt project. nop" SOURCES ${Resources_All})

# make qml visible in qtcreator
#file(GLOB_RECURSE UPNS_QMLFILES ${PROJECT_SOURCE_DIR}/*.qml ${PROJECT_SOURCE_DIR}/*/qmldir ${PROJECT_SOURCE_DIR}/*.properties)

#add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add qml files to project" SOURCES ${UPNS_QMLFILES})
