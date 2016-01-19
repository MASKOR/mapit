project(upns_visualization_renderview)
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

use_protobuf()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)

include(FindProcessorCount)
message(STATUS "used number of processor cores: " ${PROCESSOR_COUNT})

include_directories(${YAML_CPP_INCLUDE})

find_package(Qt5Qml 5.5 REQUIRED)
find_package(Qt5Widgets 5.5 REQUIRED)
find_package(Qt5Quick 5.5 REQUIRED)


if(NOT (DISABLE_PCL))
#find_package(PCL REQUIRED COMPONENTS common io apps filters kdtree search features surface keypoints registration)
find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
else( NOT (DISABLE_PCL))
endif (NOT (DISABLE_PCL))

set(UPNS_VISUALIZATION_INCLUDE ${PROJECT_SOURCE_DIR}/include)

include_directories(${PROJECT_SOURCE_DIR}/src/)

QT5_ADD_RESOURCES(UPNS_VIS_RESOURCES_RCC qml.qrc)

file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)
file(GLOB_RECURSE UPNS_SHADER ${PROJECT_SOURCE_DIR}/data/*)
file(GLOB_RECURSE UPNS_VIZ_PUBLIC_HEADER ${UPNS_VISUALIZATION_INCLUDE}/**.h)

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

add_library(${PROJECT_NAME}  ${SRC_LIST}
                                ${UPNS_SOURCES}
                                ${UPNS_VIS_RESOURCES_RCC}
                                ${FragmentShader}
                                ${VertexShader}
                                ${TesControlShader}
                                ${TesEvalShader}
                                ${GeometryShader}
                                ${GlslIncludeShader}
                                ${PCL_CUSTOM_SRC}
                                ${UPNS_VIZ_PUBLIC_HEADER})

add_custom_target(resourceFilesVizRenderview ALL ${CMAKE_COMMAND} -E echo "nop" SOURCES ${Datas_All})

if(WIN32)
#target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick ${Boost_LIBRARIES} uuid ${libzip} ${PCL_LIBRARIES} ${PCL_CUSTOM_LIB} ${OPENVDB_MINGW_BUILD_DIR}/libopenvdb.a C:/MinGW/bin/zlib1.dll ${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib/libboost_system-mgw49-mt-1_58.a ${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib/libboost_iostreams-mgw49-mt-1_58.a nulibz Half tbb ${BOOST_LIBRARIES_ADDITION})
target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${PCL_CUSTOM_LIB} ${BOOST_LIBRARIES_ADDITION})

else(WIN32)
  target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick ${Boost_LIBRARIES} ${libzip} ${PCL_LIBRARIES} ${PCL_CUSTOM_LIB})
endif(WIN32)

target_link_libraries(${PROJECT_NAME} upns_mapmanager)
target_include_directories(${PROJECT_NAME} PUBLIC ${UPNS_VISUALIZATION_INCLUDE})

file(GLOB_RECURSE UPNS_QMLFILES ${PROJECT_SOURCE_DIR}/*.qml ${PROJECT_SOURCE_DIR}/*.js ${PROJECT_SOURCE_DIR}/*/qmldir)

add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add qml files to project" SOURCES ${UPNS_QMLFILES})

#TODO: some are not needed anymore
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/data ${CMAKE_BINARY_DIR}/tools/visualization/data
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/test/unit_tests/libs
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/resources ${CMAKE_BINARY_DIR}/tools/visualization/resources
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/libs/operator_modules ${CMAKE_BINARY_DIR}/tools/visualization/operator_modules
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/libs/layertypes_collection ${CMAKE_BINARY_DIR}/tools/visualization/layertypes_collection
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/logging.properties ${CMAKE_BINARY_DIR}/test/unit_tests/logging.properties
)
