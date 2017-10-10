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

find_package(Boost COMPONENTS program_options REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)

include(FindProcessorCount)
message(STATUS "used number of processor cores: " ${PROCESSOR_COUNT})

find_package(Qt5Core REQUIRED)
find_package(Qt5Qml REQUIRED)
#find_package(Qt5Widgets REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt53DCore REQUIRED)
find_package(Qt53DInput REQUIRED)
find_package(Qt53DLogic REQUIRED)
find_package(Qt53DQuick REQUIRED)
find_package(Qt53DRender REQUIRED)
find_package(Qt53DQuickInput REQUIRED)
find_package(Qt53DQuickRender REQUIRED)

#if(WITH_PCL)
#    #find_package(PCL REQUIRED COMPONENTS common io filters kdtree search features surface keypoints registration octree sample_consensus segmentation tracking) # apps visualization)
#    find_package(PCL REQUIRED COMPONENTS common io) # apps visualization)

#    #find_package(PCL REQUIRED COMPONENTS common io search kdtree features filters)
#    include_directories(${PCL_INCLUDE_DIRS})
#    link_directories(${PCL_LIBRARY_DIRS})
#    add_definitions(${PCL_DEFINITIONS})
#else(WITH_PCL)
#endif(WITH_PCL)

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

#hardcoded for ubuntu 16.04 + qt3d input/logic should not affect other installations
link_directories("/opt/qt59/lib")

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


target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/src/
                                                   ${PROJECT_SOURCE_DIR}/../Qt3DPointcloudRenderer/include)

if(WIN32)
#target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick ${Boost_LIBRARIES} uuid ${libzip} ${PCL_LIBRARIES} ${OPENVDB_MINGW_BUILD_DIR}/libopenvdb.a C:/MinGW/bin/zlib1.dll ${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib/libboost_system-mgw49-mt-1_58.a ${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib/libboost_iostreams-mgw49-mt-1_58.a nulibz Half tbb)
target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick Qt5::WinMain Xinput9_1_0)
    #set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS_RELEASE "/SUBSYSTEM:WINDOWS")
else(WIN32)
  target_link_libraries(${PROJECT_NAME} ${libzip})
endif(WIN32)

qt5_use_modules(${PROJECT_NAME} Qml Quick 3DCore 3DInput 3DLogic 3DQuick 3DRender 3DQuickInput 3DQuickRender)

target_link_libraries(${PROJECT_NAME} upns_visualization
    ${mapit_core}
    ${mapit_interface_cpp}
    ${mapit_standard_repository_factory}
    Qt3DPointcloudRenderer
    layertype_tf
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    stdc++)

######## FOR DEBUGGING PURPOSES ONLY ########

#find_package(OpenVDB REQUIRED)
#target_link_libraries(${PROJECT_NAME} load_las_from_csv)

#target_link_libraries(${PROJECT_NAME} load_las_from_csv)
#target_link_libraries(${PROJECT_NAME} load_asset)
#target_link_libraries(${PROJECT_NAME} surfrecon_openvdb ${OPENVDB_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} layertype_openvdb)
#target_link_libraries(${PROJECT_NAME} levelset_to_mesh)
#target_link_libraries(${PROJECT_NAME} las2pcd)

#############################################

add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add qml files to project" SOURCES ${UPNS_QMLFILES})

if(WIN32)
    message("Please Create Symlinc MANUALLY (see visualization_standalone cmake:95")
    # Windows will not let us create symlinks:
    # 1) cmake does not have a platform agnostic way
    # 2) usermanagement: user has not the privileges to create a symlink
    # 3) backslashes have to be done manually
    # please execute following lines manually once after first build, with administrative privileges in the lovely win cmd:
    # (works if build folder is next to sources)
    # mklink /D data ..\..\..\upns_software\data
    # mklink /D libs ..\..\libs
    # mklink /D resources ..\..\..\upns_software\resources
    # mklink logging.properties ..\..\..\upns_software\logging.properties

#    STRING(REGEX REPLACE "/" "\\\\" BINARY_PATH_TMP_WINDOWS ${CMAKE_BINARY_DIR})
#    STRING(REGEX REPLACE "/" "\\\\" UPNS_ROOT_DIR_TMP_WINDOWS ${UPNS_ROOT_DIR})
#    execute_process(COMMAND CMD /c echo /D "${BINARY_PATH_TMP_WINDOWS}\\tools\\visualization_standalone\\data" "${UPNS_ROOT_DIR_TMP_WINDOWS}\\data")
#    execute_process(COMMAND CMD /c mklink /D "\"${BINARY_PATH_TMP_WINDOWS}\\tools\\visualization_standalone\\data\"" "\"${UPNS_ROOT_DIR_TMP_WINDOWS}\\data\"")
#    execute_process(COMMAND CMD /c mklink /D "${BINARY_PATH_TMP_WINDOWS}\\tools\\visualization_standalone\\libs" "${BINARY_PATH_TMP_WINDOWS}\\libs")
#    execute_process(COMMAND CMD /c mklink /D "${BINARY_PATH_TMP_WINDOWS}\\tools\\visualization_standalone\\resources" "${UPNS_ROOT_DIR_TMP_WINDOWS}\\resources")
#    execute_process(COMMAND CMD /c mklink /D "${BINARY_PATH_TMP_WINDOWS}\\tools\\visualization_standalone\\operator_modules" "${UPNS_ROOT_DIR_TMP_WINDOWS}\\libs\\operator_modules")
#    execute_process(COMMAND CMD /c mklink /D "${BINARY_PATH_TMP_WINDOWS}\\tools\\visualization_standalone\\layertypes_collection" "${UPNS_ROOT_DIR_TMP_WINDOWS}\\libs\\layertypes_collection")
#    execute_process(COMMAND CMD /c mklink    "${BINARY_PATH_TMP_WINDOWS}\\tools\\visualization_standalone\\logging.properties" "${UPNS_ROOT_DIR_TMP_WINDOWS}\\logging.properties")
else(WIN32)
    #TODO: some are not needed anymore
    #Note: Thought CMake can do symlink platform independent? Well... this will only work for unix...

    execute_process(
        COMMAND "${CMAKE_COMMAND}" "-E" "make_directory" ${CMAKE_BINARY_DIR}/tools/bin
    )

    execute_process(
      COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/tools/${PROJECT_NAME}/libs
    )
    execute_process(
      COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${PROJECT_SOURCE_DIR}/src/repo.yaml ${CMAKE_BINARY_DIR}/tools/${PROJECT_NAME}/repo.yaml
    )

    execute_process(
      COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/tools/bin/libs
    )
    execute_process(
      COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${PROJECT_SOURCE_DIR}/src/repo.yaml ${CMAKE_BINARY_DIR}/tools/bin/repo.yaml
    )
    execute_process(
        COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/tools/${PROJECT_NAME}/${PROJECT_NAME} ${CMAKE_BINARY_DIR}/tools/bin/${PROJECT_NAME}
    )

endif(WIN32)

install(TARGETS ${PROJECT_NAME} DESTINATION bin/upns_tools)
