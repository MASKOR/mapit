################################################################################
#
# Copyright 2015-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
#           2015-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
#                2017 Marcus Meeßen	<marcus.meessen@alumni.fh-aachen.de>
#
################################################################################

#  This file is part of mapit.
#
#  Mapit is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  Mapit is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
#

project(visualization)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

# other libs don't like 'foreach' (openvdb?)
add_definitions(-DQT_NO_KEYWORDS)

#if(UNIX)
#set (CMAKE_PREFIX_PATH ${QTDIR}"/gcc_64")
#endif(UNIX)

# add custom cmake scripts. i.e. FindProcessorCount...
set(CMAKE_MODULE_PATH
    ${PROJECT_SOURCE_DIR}/cmake
    ${CMAKE_MODULE_PATH}
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)

include(FindProcessorCount)
message(STATUS "used number of processor cores: " ${PROCESSOR_COUNT})

find_package(Qt5Qml REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Quick REQUIRED)

string( TOLOWER "${CMAKE_BUILD_TYPE}" LOWER_CMAKE_BUILD_TYPE )
if (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
    set(OCULUSVR_LIBRARIES ${MAPIT_ROOT_DIR}/externals/windows/lib-msvc2013-md-64/LibOVR_debug.lib ${MAPIT_ROOT_DIR}/externals/windows/lib-msvc2013-md-64/LibOVRKernel_debug.lib)
else (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
    set(OCULUSVR_LIBRARIES ${MAPIT_ROOT_DIR}/externals/windows/lib-msvc2013-md-64/LibOVR_release.lib ${MAPIT_ROOT_DIR}/externals/windows/lib-msvc2013-md-64/LibOVRKernel_release.lib)
endif (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")


if(MAPIT_ENABLE_PCL)
    #find_package(PCL QUIET REQUIRED COMPONENTS common io apps filters kdtree search features surface keypoints registration)
    find_package(PCL QUIET REQUIRED COMPONENTS common io)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
else(MAPIT_ENABLE_PCL)
endif(MAPIT_ENABLE_PCL)

set(MAPIT_VISUALIZATION_INCLUDE ${PROJECT_SOURCE_DIR}/include)

include_directories(${PROJECT_SOURCE_DIR}/src/)
#include_directories(${PROJECT_SOURCE_DIR}/include)
if(WIN32)
    include_directories(${MAPIT_ROOT_DIR}/externals/windows/include_ovr)
    link_directories(${MAPIT_ROOT_DIR}/externals/windows/lib-msvc2013-md-64)
endif()

QT5_ADD_RESOURCES(VIS_RESOURCES_RCC mapit_visualization.qrc)

file(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)
file(GLOB_RECURSE SHADER ${PROJECT_SOURCE_DIR}/data/*)
file(GLOB_RECURSE VIS_PUBLIC_HEADER ${MAPIT_VISUALIZATION_INCLUDE}/**.h)

set(ShaderPath ${PROJECT_SOURCE_DIR}/data/shader/)
file(GLOB FragmentShader ${ShaderPath}/*.frag)
file(GLOB VertexShader ${ShaderPath}/*.vert)
file(GLOB TesControlShader ${ShaderPath}/*.tcs)
file(GLOB TesEvalShader ${ShaderPath}/*.tes)
file(GLOB GeometryShader ${ShaderPath}/*.geom)
file(GLOB GlslIncludeShader ${ShaderPath}/*.glsl)

set(DataDir ${PROJECT_SOURCE_DIR}/data)
file(GLOB Datas ${DataDir}/*)
file(GLOB_RECURSE Datas_All ${DataDir}/*)

add_library(${PROJECT_NAME} SHARED ${SRC_LIST}
                                ${SOURCES}
                                ${VIS_RESOURCES_RCC}
                                ${FragmentShader}
                                ${VertexShader}
                                ${TesControlShader}
                                ${TesEvalShader}
                                ${GeometryShader}
                                ${GlslIncludeShader}
                                ${VIS_PUBLIC_HEADER})

add_custom_target(resourceFilesVizRenderview ALL ${CMAKE_COMMAND} -E echo "nop" SOURCES ${Datas_All})

#target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick ${Boost_LIBRARIES} ${PCL_LIBRARIES})

if(WIN32)
#target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick ${Boost_LIBRARIES} uuid ${libzip} ${PCL_LIBRARIES} ${OPENVDB_MINGW_BUILD_DIR}/libopenvdb.a C:/MinGW/bin/zlib1.dll ${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib/libboost_system-mgw49-mt-1_58.a ${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib/libboost_iostreams-mgw49-mt-1_58.a nulibz Half tbb)
target_link_libraries(${PROJECT_NAME} ${OCULUSVR_LIBRARIES})
else(WIN32)
  target_link_libraries(${PROJECT_NAME} ${libzip})
endif(WIN32)

find_package(mapit REQUIRED)

target_link_libraries(${PROJECT_NAME}   ${mapit_core}
                                        ${mapit_interface}
                                        ${mapit_interface_cpp}
                                        standard_repository_factory
                                        Qt3DPointcloudRenderer
                                        Qt3DOpenVDBRenderer
                                        layertype_tf
                                        layertype_pose_path
                                        layertype_asset
                                        layertype_openvdb
                                        layertype_primitive
                                        networking_node
                                        Qt5::Qml Qt5::Quick
                                        ${Boost_LIBRARIES}
                                        stdc++)
if(MAPIT_ENABLE_PCL)
    target_link_libraries(${PROJECT_NAME}
                                        Qt3DPointcloudRenderer
                                        layertype_pointcloud2
                                        ${PCL_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME} PUBLIC MAPIT_ENABLE_PCL=1)
endif(MAPIT_ENABLE_PCL)

if(WITH_LAS)
    target_compile_definitions(${PROJECT_NAME} PUBLIC WITH_LAS=1)
    target_link_libraries(${PROJECT_NAME}
                                        Qt3DPointcloudRenderer
                                        layertype_las)
endif(WITH_LAS)

target_include_directories(${PROJECT_NAME} PUBLIC  ${MAPIT_VISUALIZATION_INCLUDE}
                                           PRIVATE ${PROJECT_SOURCE_DIR}/include/bindings
#                                                   ${PROJECT_SOURCE_DIR}/include/tools
                                                   ${PROJECT_SOURCE_DIR}/../Qt3DPointcloudRenderer/include
                                                   ${PROJECT_SOURCE_DIR}/../Qt3DOpenVDBRenderer/include)


file(GLOB_RECURSE QMLFILES ${PROJECT_SOURCE_DIR}/*.qml ${PROJECT_SOURCE_DIR}/*.js ${PROJECT_SOURCE_DIR}/*/qmldir)

add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add qml files to project" SOURCES ${QMLFILES})
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${CMAKE_LIBRARY_ARCHITECTURE})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${CMAKE_LIBRARY_ARCHITECTURE} )


qt5_use_modules(${PROJECT_NAME} Qml Quick 3DCore 3DInput 3DLogic 3DQuick 3DRender 3DExtras 3DQuickInput 3DQuickRender)

#TODO: some are not needed anymore
if(WIN32)
else()
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${MAPIT_ROOT_DIR}/data ${CMAKE_BINARY_DIR}/tools/visualization/data
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/test/unit_tests/libs
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${MAPIT_ROOT_DIR}/resources ${CMAKE_BINARY_DIR}/tools/visualization/resources
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${MAPIT_ROOT_DIR}/libs/operator_modules ${CMAKE_BINARY_DIR}/tools/visualization/operator_modules
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${MAPIT_ROOT_DIR}/libs/layertypes_collection ${CMAKE_BINARY_DIR}/tools/visualization/layertypes_collection
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${MAPIT_ROOT_DIR}/logging.properties ${CMAKE_BINARY_DIR}/test/unit_tests/logging.properties
#)
endif()
