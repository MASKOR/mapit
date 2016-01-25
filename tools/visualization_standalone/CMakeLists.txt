project(upns_visualization_standalone)
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

find_package(Qt5Core 5.5 REQUIRED)
find_package(Qt5Core 5.5 REQUIRED)
find_package(Qt5Qml 5.5 REQUIRED)
find_package(Qt5Widgets 5.5 REQUIRED)
find_package(Qt5Quick 5.5 REQUIRED)


if(NOT (DISABLE_PCL))
#find_package(PCL REQUIRED COMPONENTS common io apps filters kdtree search features surface keypoints registration)
find_package(PCL REQUIRED COMPONENTS common io search kdtree features)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
else( NOT (DISABLE_PCL))
endif (NOT (DISABLE_PCL))

include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${UPNS_VISUALIZATION_INCLUDE})

QT5_ADD_RESOURCES(UPNS_VIS_RESOURCES_RCC qml.qrc)

file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)
file(GLOB_RECURSE UPNS_SHADER ${PROJECT_SOURCE_DIR}/data/*)

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

add_executable(${PROJECT_NAME}  ${SRC_LIST}
                                ${UPNS_SOURCES}
                                ${UPNS_VIS_RESOURCES_RCC}
                                ${FragmentShader}
                                ${VertexShader}
                                ${TesControlShader}
                                ${TesEvalShader}
                                ${GeometryShader}
                                ${GlslIncludeShader}
                                ${PCL_CUSTOM_SRC})

add_custom_target(resourceFilesVizStandalone ALL ${CMAKE_COMMAND} -E echo "nop" SOURCES ${Datas_All})

if(WIN32)
#target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick ${Boost_LIBRARIES} uuid ${libzip} ${PCL_LIBRARIES} ${PCL_CUSTOM_LIB} ${OPENVDB_MINGW_BUILD_DIR}/libopenvdb.a C:/MinGW/bin/zlib1.dll ${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib/libboost_system-mgw49-mt-1_58.a ${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib/libboost_iostreams-mgw49-mt-1_58.a nulibz Half tbb ${BOOST_LIBRARIES_ADDITION})
target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick Qt5::WinMain ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${PCL_CUSTOM_LIB} ${BOOST_LIBRARIES_ADDITION}
    C:/boost_1_56_0/lib64-msvc-12.0/libboost_system-vc120-mt-gd-1_56.lib
    C:/boost_1_56_0/lib64-msvc-12.0/libboost_date_time-vc120-mt-gd-1_56.lib)
    #set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS_RELEASE "/SUBSYSTEM:WINDOWS")
else(WIN32)
  target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick ${Boost_LIBRARIES} ${libzip} ${PCL_LIBRARIES} ${PCL_CUSTOM_LIB})
endif(WIN32)

target_link_libraries(${PROJECT_NAME} upns_visualization_renderview upns_mapmanager)

file(GLOB_RECURSE UPNS_QMLFILES ${PROJECT_SOURCE_DIR}/*.qml ${PROJECT_SOURCE_DIR}/*.js ${PROJECT_SOURCE_DIR}/*/qmldir)

add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add qml files to project" SOURCES ${UPNS_QMLFILES})

if(WIN32)
    message("Please Create Symlinc MANUALLY (see visualization_standalone cmake:93")
    # Steps, Windows takes to not let us create symlinks:
    # 1) cmake does not have a platform agnostic way
    # 2) usermanagement: user has not the privileges to create a stupid symlink
    # 3) backslashes have to be done manually
    # The words windows and developer do just not fit together.
    # please execute following lines manually with administrative privileges in the lovely cmd:
    # (works if build folder is next to sources)
    # mklink /D data ..\..\..\upns_software\data
    # mklink /D libs ..\..\libs
    # mklink /D resources ..\..\..\upns_software\resources
    # mklink logging.properties ..\..\..\upns_software\logging.properties
    STRING(REGEX REPLACE "/" "\\\\" BINARY_PATH_SHITTY_WINDOWS ${CMAKE_BINARY_DIR})
    STRING(REGEX REPLACE "/" "\\\\" UPNS_ROOT_DIR_SHITTY_WINDOWS ${UPNS_ROOT_DIR})
    execute_process(COMMAND CMD /c echo /D "${BINARY_PATH_SHITTY_WINDOWS}\\tools\\visualization_standalone\\data" "${UPNS_ROOT_DIR_SHITTY_WINDOWS}\\data")
    execute_process(COMMAND CMD /c mklink /D "\"${BINARY_PATH_SHITTY_WINDOWS}\\tools\\visualization_standalone\\data\"" "\"${UPNS_ROOT_DIR_SHITTY_WINDOWS}\\data\"")
    execute_process(COMMAND CMD /c mklink /D "${BINARY_PATH_SHITTY_WINDOWS}\\tools\\visualization_standalone\\libs" "${BINARY_PATH_SHITTY_WINDOWS}\\libs")
    execute_process(COMMAND CMD /c mklink /D "${BINARY_PATH_SHITTY_WINDOWS}\\tools\\visualization_standalone\\resources" "${UPNS_ROOT_DIR_SHITTY_WINDOWS}\\resources")
    execute_process(COMMAND CMD /c mklink /D "${BINARY_PATH_SHITTY_WINDOWS}\\tools\\visualization_standalone\\operator_modules" "${UPNS_ROOT_DIR_SHITTY_WINDOWS}\\libs\\operator_modules")
    execute_process(COMMAND CMD /c mklink /D "${BINARY_PATH_SHITTY_WINDOWS}\\tools\\visualization_standalone\\layertypes_collection" "${UPNS_ROOT_DIR_SHITTY_WINDOWS}\\libs\\layertypes_collection")
    execute_process(COMMAND CMD /c mklink    "${BINARY_PATH_SHITTY_WINDOWS}\\tools\\visualization_standalone\\logging.properties" "${UPNS_ROOT_DIR_SHITTY_WINDOWS}\\logging.properties")
else(WIN32)
    #TODO: some are not needed anymore
    #Note: Thought CMake can do symlink platform independent? Well... this will only work for unix...
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/data ${CMAKE_BINARY_DIR}/tools/visualization_standalone/data
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/tools/visualization_standalone/libs
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/resources ${CMAKE_BINARY_DIR}/tools/visualization_standalone/resources
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/libs/operator_modules ${CMAKE_BINARY_DIR}/tools/visualization_standalone/operator_modules
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/libs/layertypes_collection ${CMAKE_BINARY_DIR}/tools/visualization_standalone/layertypes_collection
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/logging.properties ${CMAKE_BINARY_DIR}/tools/visualization_standalone/logging.properties
)
endif(WIN32)
