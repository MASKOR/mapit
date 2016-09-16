project(visualization_standalone)
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

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)

include(FindProcessorCount)
message(STATUS "used number of processor cores: " ${PROCESSOR_COUNT})

if(WIN32)
    set(YAML_CPP_LIBRARIES debug "${YAML_CPP_LIBRARY_DEBUG}" optimized "${YAML_CPP_LIBRARY_RELEASE}")
else(WIN32)
    set(YAML_CPP_LIBRARIES "${YAML_CPP_LIBRARY_DEBUG}" "${YAML_CPP_LIBRARY_RELEASE}")
endif(WIN32)

include_directories(${YAML_CPP_INCLUDE})

find_package(Qt5Core 5.6 REQUIRED)
find_package(Qt5Qml 5.6 REQUIRED)
find_package(Qt5Widgets 5.6 REQUIRED)
find_package(Qt5Quick 5.6 REQUIRED)
find_package(Qt53DCore REQUIRED)
find_package(Qt53DInput REQUIRED)
find_package(Qt53DLogic REQUIRED)
find_package(Qt53DQuick REQUIRED)
find_package(Qt53DRender REQUIRED)
find_package(Qt53DQuickInput REQUIRED)
find_package(Qt53DQuickRender REQUIRED)

if(NOT (DISABLE_PCL))
    find_package(PCL REQUIRED COMPONENTS common io apps filters kdtree search features surface keypoints registration octree sample_consensus segmentation tracking) # apps visualization)
#find_package(PCL REQUIRED COMPONENTS common io search kdtree features filters)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
else( NOT (DISABLE_PCL))
endif (NOT (DISABLE_PCL))

include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${UPNS_VISUALIZATION_INCLUDE})
include_directories(${PROJECT_SOURCE_DIR}/../Qt3DPointcloudRenderer/include)

QT5_ADD_RESOURCES(UPNS_VIS_RESOURCES_RCC qml.qrc)

file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)
file(GLOB_RECURSE UPNS_SHADER ${PROJECT_SOURCE_DIR}/data/*)

set(ShaderPath ${PROJECT_SOURCE_DIR}/data/shader/ ${PROJECT_SOURCE_DIR}/shader/)
file(GLOB FragmentShader ${ShaderPath}/*.frag)
file(GLOB VertexShader ${ShaderPath}/*.vert)
file(GLOB TesControlShader ${ShaderPath}/*.tcs)
file(GLOB TesEvalShader ${ShaderPath}/*.tes)
file(GLOB GeometryShader ${ShaderPath}/*.geom)
file(GLOB GlslIncludeShader ${ShaderPath}/*.glsl)

set(DataDir ${PROJECT_SOURCE_DIR}/data)
file(GLOB Datas ${DataDir}/*)
file(GLOB_RECURSE Datas_All ${DataDir}/*)

file(GLOB_RECURSE UPNS_QMLFILES ${PROJECT_SOURCE_DIR}/*.qml ${PROJECT_SOURCE_DIR}/*.js ${PROJECT_SOURCE_DIR}/*/qmldir)

add_executable(${PROJECT_NAME}  ${SRC_LIST}
                                ${UPNS_SOURCES}
                                ${UPNS_VIS_RESOURCES_RCC}
                                ${FragmentShader}
                                ${VertexShader}
                                ${TesControlShader}
                                ${TesEvalShader}
                                ${GeometryShader}
                                ${GlslIncludeShader}

                                ${UPNS_QMLFILES})

add_custom_target(resourceFilesVizStandalone ALL ${CMAKE_COMMAND} -E echo "nop" SOURCES ${Datas_All})

#custom_target_use_boost(${PROJECT_NAME})
find_package(Boost COMPONENTS filesystem system date_time)

if(WIN32)
#target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick ${Boost_LIBRARIES} uuid ${libzip} ${PCL_LIBRARIES} ${OPENVDB_MINGW_BUILD_DIR}/libopenvdb.a C:/MinGW/bin/zlib1.dll ${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib/libboost_system-mgw49-mt-1_58.a ${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib/libboost_iostreams-mgw49-mt-1_58.a nulibz Half tbb)
target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick Qt5::WinMain Xinput9_1_0 ${Boost_LIBRARIES} ${PCL_LIBRARIES})
    #set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS_RELEASE "/SUBSYSTEM:WINDOWS")
else(WIN32)
  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${libzip} ${PCL_LIBRARIES})
endif(WIN32)

qt5_use_modules(${PROJECT_NAME} Qml Quick 3DCore 3DInput 3DLogic 3DQuick 3DRender 3DQuickInput 3DQuickRender)

target_link_libraries(${PROJECT_NAME} upns_visualization_renderview upns_mapmanager Qt3DPointcloudRenderer layertype_tf ${YAML_CPP_LIBRARIES})


add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add qml files to project" SOURCES ${UPNS_QMLFILES})

if(WIN32)
    message("Please Create Symlinc MANUALLY (see visualization_standalone cmake:95")
    # Steps, Windows takes to not let us create symlinks:
    # 1) cmake does not have a platform agnostic way
    # 2) usermanagement: user has not the privileges to create a stupid symlink
    # 3) backslashes have to be done manually
    # The words windows and developer do just not fit together.
    # please execute following lines manually once after first build, with administrative privileges in the lovely win cmd:
    # (works if build folder is next to sources)
    # mklink /D data ..\..\..\upns_software\data
    # mklink /D libs ..\..\libs
    # mklink /D resources ..\..\..\upns_software\resources
    # mklink logging.properties ..\..\..\upns_software\logging.properties
#    STRING(REGEX REPLACE "/" "\\\\" BINARY_PATH_SHITTY_WINDOWS ${CMAKE_BINARY_DIR})
#    STRING(REGEX REPLACE "/" "\\\\" UPNS_ROOT_DIR_SHITTY_WINDOWS ${UPNS_ROOT_DIR})
#    execute_process(COMMAND CMD /c echo /D "${BINARY_PATH_SHITTY_WINDOWS}\\tools\\visualization_standalone\\data" "${UPNS_ROOT_DIR_SHITTY_WINDOWS}\\data")
#    execute_process(COMMAND CMD /c mklink /D "\"${BINARY_PATH_SHITTY_WINDOWS}\\tools\\visualization_standalone\\data\"" "\"${UPNS_ROOT_DIR_SHITTY_WINDOWS}\\data\"")
#    execute_process(COMMAND CMD /c mklink /D "${BINARY_PATH_SHITTY_WINDOWS}\\tools\\visualization_standalone\\libs" "${BINARY_PATH_SHITTY_WINDOWS}\\libs")
#    execute_process(COMMAND CMD /c mklink /D "${BINARY_PATH_SHITTY_WINDOWS}\\tools\\visualization_standalone\\resources" "${UPNS_ROOT_DIR_SHITTY_WINDOWS}\\resources")
#    execute_process(COMMAND CMD /c mklink /D "${BINARY_PATH_SHITTY_WINDOWS}\\tools\\visualization_standalone\\operator_modules" "${UPNS_ROOT_DIR_SHITTY_WINDOWS}\\libs\\operator_modules")
#    execute_process(COMMAND CMD /c mklink /D "${BINARY_PATH_SHITTY_WINDOWS}\\tools\\visualization_standalone\\layertypes_collection" "${UPNS_ROOT_DIR_SHITTY_WINDOWS}\\libs\\layertypes_collection")
#    execute_process(COMMAND CMD /c mklink    "${BINARY_PATH_SHITTY_WINDOWS}\\tools\\visualization_standalone\\logging.properties" "${UPNS_ROOT_DIR_SHITTY_WINDOWS}\\logging.properties")
else(WIN32)
    #TODO: some are not needed anymore
    #Note: Thought CMake can do symlink platform independent? Well... this will only work for unix...
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/data ${CMAKE_BINARY_DIR}/tools/visualization_standalone/data
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/tools/visualization_standalone/libs
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/resources ${CMAKE_BINARY_DIR}/tools/visualization_standalone/resources
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/libs/operator_modules ${CMAKE_BINARY_DIR}/tools/visualization_standalone/operator_modules
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/libs/layertypes_collection ${CMAKE_BINARY_DIR}/tools/visualization_standalone/layertypes_collection
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/logging.properties ${CMAKE_BINARY_DIR}/tools/visualization_standalone/logging.properties
#)

#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${PROJECT_SOURCE_DIR}/src/repo.yaml ${CMAKE_BINARY_DIR}/tools/visualization_standalone/repo.yaml
#)

#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/data ${CMAKE_BINARY_DIR}/tools/bin/data
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/tools/bin/libs
#)
#execute_process(
#  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${PROJECT_SOURCE_DIR}/src/repo.yaml ${CMAKE_BINARY_DIR}/tools/bin/repo.yaml
#)
#execute_process(
#    COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/tools/${PROJECT_NAME}/${PROJECT_NAME} ${CMAKE_BINARY_DIR}/tools/bin/${PROJECT_NAME}
#)
endif(WIN32)

