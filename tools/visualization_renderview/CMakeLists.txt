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

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)

include(FindProcessorCount)
message(STATUS "used number of processor cores: " ${PROCESSOR_COUNT})

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

#find_package(Qt5Qml 5.5 REQUIRED)
#find_package(Qt5Widgets 5.5 REQUIRED)
#find_package(Qt5Quick 5.5 REQUIRED)
find_package(Qt5Qml REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Quick REQUIRED)

string( TOLOWER "${CMAKE_BUILD_TYPE}" LOWER_CMAKE_BUILD_TYPE )
if (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
    set(OCULUSVR_LIBRARIES ${UPNS_ROOT_DIR}/externals/windows/lib-msvc2013-md-64/LibOVR_debug.lib ${UPNS_ROOT_DIR}/externals/windows/lib-msvc2013-md-64/LibOVRKernel_debug.lib)
else (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
    set(OCULUSVR_LIBRARIES ${UPNS_ROOT_DIR}/externals/windows/lib-msvc2013-md-64/LibOVR_release.lib ${UPNS_ROOT_DIR}/externals/windows/lib-msvc2013-md-64/LibOVRKernel_release.lib)
endif (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")


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
if(WIN32)
    include_directories(${UPNS_ROOT_DIR}/externals/windows/include_ovr)
    link_directories(${UPNS_ROOT_DIR}/externals/windows/lib-msvc2013-md-64)
endif()

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

include_directories(${PROJECT_SOURCE_DIR}/include/bindings)
include_directories(${PROJECT_SOURCE_DIR}/../Qt3DPointcloudRenderer/include)

add_library(${PROJECT_NAME}  ${SRC_LIST}
                                ${UPNS_SOURCES}
                                ${UPNS_VIS_RESOURCES_RCC}
                                ${FragmentShader}
                                ${VertexShader}
                                ${TesControlShader}
                                ${TesEvalShader}
                                ${GeometryShader}
                                ${GlslIncludeShader}

                                ${UPNS_VIZ_PUBLIC_HEADER})

add_custom_target(resourceFilesVizRenderview ALL ${CMAKE_COMMAND} -E echo "nop" SOURCES ${Datas_All})

if(WIN32)
#target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick ${Boost_LIBRARIES} uuid ${libzip} ${PCL_LIBRARIES} ${OPENVDB_MINGW_BUILD_DIR}/libopenvdb.a C:/MinGW/bin/zlib1.dll ${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib/libboost_system-mgw49-mt-1_58.a ${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib/libboost_iostreams-mgw49-mt-1_58.a nulibz Half tbb)
target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick ${Boost_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OCULUSVR_LIBRARIES})
else(WIN32)
  target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick ${Boost_LIBRARIES} ${libzip} ${PCL_LIBRARIES})
endif(WIN32)

target_link_libraries(${PROJECT_NAME} upns_mapmanager upns_interface upns_interface_cpp Qt3DPointcloudRenderer layertype_pointcloud2)
target_include_directories(${PROJECT_NAME} PUBLIC ${UPNS_VISUALIZATION_INCLUDE})

file(GLOB_RECURSE UPNS_QMLFILES ${PROJECT_SOURCE_DIR}/*.qml ${PROJECT_SOURCE_DIR}/*.js ${PROJECT_SOURCE_DIR}/*/qmldir)

add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add qml files to project" SOURCES ${UPNS_QMLFILES})

#TODO: some are not needed anymore
if(WIN32)
else()
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/data ${CMAKE_BINARY_DIR}/tools/visualization/data
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/test/unit_tests/libs
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/resources ${CMAKE_BINARY_DIR}/tools/visualization/resources
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/libs/operator_modules ${CMAKE_BINARY_DIR}/tools/visualization/operator_modules
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/libs/layertypes_collection ${CMAKE_BINARY_DIR}/tools/visualization/layertypes_collection
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/logging.properties ${CMAKE_BINARY_DIR}/test/unit_tests/logging.properties
#)
endif()
