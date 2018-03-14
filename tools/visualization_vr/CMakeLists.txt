################################################################################
#
# Copyright 2016-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
#                2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

project(visualization_vr)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

#TODO: Clean up CMake after meetintg

remove_definitions(-DQT_NO_KEYWORDS)

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

set(OPENVR_INSTALL_DIR CACHE PATH "OpenVR Installation directory with headers and lib/linux64")
include_directories( ${OPENVR_INSTALL_DIR}/headers )
link_directories( ${OPENVR_INSTALL_DIR}/lib/linux64 )

include_directories( ${Qt5Quick_INCLUDE_DIRS} )
include_directories( ${Qt5Quick_PRIVATE_INCLUDE_DIRS} )
set(QT_PRIVATE_HEADERS "/usr/include/qt" CACHE PATH "Point to qt installation dir where private headers can be found. This can not be done via find-script, e.g. /usr/include/qt")

if(NOT EXISTS "${QT_PRIVATE_HEADERS}/Qt3DRender/5.10.1/Qt3DRender/private/qrenderaspect_p.h" )
  message("Private Headers for virtual reality not found. Disable MAPIT_ENABLE_VIRTUALREALITY or set QT_PRIVATE_HEADERS")
endif(NOT EXISTS "${QT_PRIVATE_HEADERS}/Qt3DRender/5.10.1/Qt3DRender/private/qrenderaspect_p.h")


include_directories( ${QT_PRIVATE_HEADERS}/Qt3DCore/5.10.1/ ) # for private headers
include_directories( ${QT_PRIVATE_HEADERS}/Qt3DRender/5.10.1/ ) # for private headers
include_directories( ${QT_PRIVATE_HEADERS}/Qt3DCore/5.10.1/Qt3DCore ) # for private headers
include_directories( ${QT_PRIVATE_HEADERS}/Qt3DRender/5.10.1/Qt3DRender ) # for private headers

add_definitions(-DQT3DVR_COMPILE_WITH_OPENVR=true)

#set(QtPrivateHeaderRoot )

#if(MAPIT_ENABLE_PCL)
#    #find_package(PCL QUIET REQUIRED COMPONENTS common io filters kdtree search features surface keypoints registration octree sample_consensus segmentation tracking) # apps visualization)
#    find_package(PCL QUIET REQUIRED COMPONENTS common io) # apps visualization)

#    #find_package(PCL QUIET REQUIRED COMPONENTS common io search kdtree features filters)
#    include_directories(${PCL_INCLUDE_DIRS})
#    link_directories(${PCL_LIBRARY_DIRS})
#    add_definitions(${PCL_DEFINITIONS})
#else(MAPIT_ENABLE_PCL)
#endif(MAPIT_ENABLE_PCL)

QT5_ADD_RESOURCES(VIS_RESOURCES_RCC mapit_visualization_vr.qrc)

file(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)
#file(GLOB_RECURSE SHADER ${PROJECT_SOURCE_DIR}/data/*)

file(GLOB_RECURSE QT3DVR_SOURCES ${PROJECT_SOURCE_DIR}/qt3d-vr-example/virtualreality/*.cpp ${PROJECT_SOURCE_DIR}/qt3d-vr-example/virtualreality/*.h)

#file(GLOB_RECURSE QT3DVR_SOURCES ${PROJECT_SOURCE_DIR}/../qt3d-vr-example/virtualreality/*.cpp ${PROJECT_SOURCE_DIR}/../qt3d-vr-example/virtualreality/*.h)
#list(REMOVE_ITEM QT3DVR_SOURCES ${PROJECT_SOURCE_DIR}/../qt3d-vr-example/virtualreality/vrbackends/ovr/*)
#list(REMOVE_ITEM QT3DVR_SOURCES ${PROJECT_SOURCE_DIR}/../qt3d-vr-example/virtualreality/vrbackends/openvr/*)

#set(ShaderPath ${PROJECT_SOURCE_DIR}/data/shader/ ${PROJECT_SOURCE_DIR}/shader/)
#file(GLOB FragmentShader ${ShaderPath}/*.frag)
#file(GLOB VertexShader ${ShaderPath}/*.vert)
#file(GLOB TesControlShader ${ShaderPath}/*.tcs)
#file(GLOB TesEvalShader ${ShaderPath}/*.tes)
#file(GLOB GeometryShader ${ShaderPath}/*.geom)
#file(GLOB GlslIncludeShader ${ShaderPath}/*.glsl)

#set(DataDir ${PROJECT_SOURCE_DIR}/data)
#file(GLOB Datas ${DataDir}/*)
#file(GLOB_RECURSE Datas_All ${DataDir}/*)

#file(GLOB_RECURSE QMLFILES ${PROJECT_SOURCE_DIR}/*.qml ${PROJECT_SOURCE_DIR}/*.js ${PROJECT_SOURCE_DIR}/*/qmldir)

add_executable(${PROJECT_NAME}  ${SRC_LIST}
                                ${QT3DVR_SOURCES}
                                ${SOURCES}
                                ${QT3DVR_SOURCES}
                                ${VIS_RESOURCES_RCC}
                                ${FragmentShader}
                                ${VertexShader}
                                ${TesControlShader}
                                ${TesEvalShader}
                                ${GeometryShader}
                                ${GlslIncludeShader}
                                ${OPENVR_INCLUDE_DIRS}
                                ${QMLFILES})

add_custom_target(resourceFilesVizVR ALL ${CMAKE_COMMAND} -E echo "nop" SOURCES ${Datas_All})

#custom_target_use_boost(${PROJECT_NAME})
find_package(Boost COMPONENTS filesystem system date_time)


target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/src/
                                                   ${PROJECT_SOURCE_DIR}/../Qt3DPointcloudRenderer/include
                                                   ${PROJECT_SOURCE_DIR}/qt3d-vr-example/virtualreality)

if(WIN32)
#target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick ${Boost_LIBRARIES} uuid ${libzip} ${PCL_LIBRARIES} ${OPENVDB_MINGW_BUILD_DIR}/libopenvdb.a C:/MinGW/bin/zlib1.dll ${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib/libboost_system-mgw49-mt-1_58.a ${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib/libboost_iostreams-mgw49-mt-1_58.a nulibz Half tbb)
target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick Qt5::WinMain Xinput9_1_0)
    #set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS_RELEASE "/SUBSYSTEM:WINDOWS")
else(WIN32)
  target_link_libraries(${PROJECT_NAME} ${libzip})
endif(WIN32)

qt5_use_modules(${PROJECT_NAME} Qml Quick 3DCore 3DInput 3DLogic 3DQuick 3DRender 3DExtras 3DQuickInput 3DQuickRender)

target_link_libraries(${PROJECT_NAME} visualization
    ${mapit_core}
    ${mapit_interface_cpp}
    ${mapit_standard_repository_factory}
    Qt3DPointcloudRenderer
    Qt3DOpenVDBRenderer
    layertype_tf
    layertype_primitive
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    openvr_api
    stdc++)

######## FOR DEBUGGING PURPOSES ONLY ########

#find_package(OpenVDB REQUIRED)
target_link_libraries(${PROJECT_NAME} load_primitive)
target_link_libraries(${PROJECT_NAME} load_tfs)
target_link_libraries(${PROJECT_NAME} reg_local)

#target_link_libraries(${PROJECT_NAME} load_las_from_csv)
#target_link_libraries(${PROJECT_NAME} load_asset)
#target_link_libraries(${PROJECT_NAME} surfrecon_openvdb ${OPENVDB_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} layertype_openvdb)
#target_link_libraries(${PROJECT_NAME} levelset_to_mesh)
#target_link_libraries(${PROJECT_NAME} las2pcd)

#############################################

add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add qml files to project" SOURCES ${QMLFILES})

if(WIN32)
    message("Please Create Symlinc MANUALLY (see visualization_standalone cmake:95")
    # Windows will not let us create symlinks:
    # 1) cmake does not have a platform agnostic way
    # 2) usermanagement: user has not the privileges to create a symlink
    # 3) backslashes have to be done manually
    # please execute following lines manually once after first build, with administrative privileges in the lovely win cmd:
    # (works if build folder is next to sources)
    # mklink /D data ..\..\..\mapit\data
    # mklink /D libs ..\..\libs
    # mklink /D resources ..\..\..\mapit\resources
    # mklink logging.properties ..\..\..\mapit\logging.properties

#    STRING(REGEX REPLACE "/" "\\\\" BINARY_PATH_TMP_WINDOWS ${CMAKE_BINARY_DIR})
#    STRING(REGEX REPLACE "/" "\\\\" MAPIT_ROOT_DIR_TMP_WINDOWS ${MAPIT_ROOT_DIR})
#    execute_process(COMMAND CMD /c echo /D "${BINARY_PATH_TMP_WINDOWS}\\tools\\visualization_standalone\\data" "${MAPIT_ROOT_DIR_TMP_WINDOWS}\\data")
#    execute_process(COMMAND CMD /c mklink /D "\"${BINARY_PATH_TMP_WINDOWS}\\tools\\visualization_standalone\\data\"" "\"${MAPIT_ROOT_DIR_TMP_WINDOWS}\\data\"")
#    execute_process(COMMAND CMD /c mklink /D "${BINARY_PATH_TMP_WINDOWS}\\tools\\visualization_standalone\\libs" "${BINARY_PATH_TMP_WINDOWS}\\libs")
#    execute_process(COMMAND CMD /c mklink /D "${BINARY_PATH_TMP_WINDOWS}\\tools\\visualization_standalone\\resources" "${MAPIT_ROOT_DIR_TMP_WINDOWS}\\resources")
#    execute_process(COMMAND CMD /c mklink /D "${BINARY_PATH_TMP_WINDOWS}\\tools\\visualization_standalone\\operator_modules" "${MAPIT_ROOT_DIR_TMP_WINDOWS}\\libs\\operator_modules")
#    execute_process(COMMAND CMD /c mklink /D "${BINARY_PATH_TMP_WINDOWS}\\tools\\visualization_standalone\\layertypes_collection" "${MAPIT_ROOT_DIR_TMP_WINDOWS}\\libs\\layertypes_collection")
#    execute_process(COMMAND CMD /c mklink    "${BINARY_PATH_TMP_WINDOWS}\\tools\\visualization_standalone\\logging.properties" "${MAPIT_ROOT_DIR_TMP_WINDOWS}\\logging.properties")
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

install(TARGETS ${PROJECT_NAME} DESTINATION libexec/mapit_tools RENAME mapit-${PROJECT_NAME})
