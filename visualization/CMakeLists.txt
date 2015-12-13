
if(USE_PCL)
project(upns_visualization)
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


find_package(Qt5Qml 5.4 REQUIRED)
find_package(Qt5Widgets 5.4 REQUIRED)
find_package(Qt5Quick 5.4 REQUIRED)

find_package(Boost REQUIRED system iostreams)
find_package(TBB REQUIRED)

# didn't do the hassle to compile pcl under windows...
if (NOT WIN32)
find_package(PCL 1.7 REQUIRED COMPONENTS common io apps filters kdtree search features surface keypoints registration)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DENABLE_PCL)
endif (NOT WIN32)

include_directories(${PROJECT_SOURCE_DIR}/src/)

# there is no FindOpenVDB - cmake script yet...
#### FindOpenVDB

if(UNIX)
add_library( openvdb SHARED IMPORTED )
set_target_properties( openvdb PROPERTIES IMPORTED_LOCATION /usr/local/lib/libopenvdb.so )
endif(UNIX)

# todo: windows part is a mess
if(WIN32)
include_directories(${TBB_INCLUDE_DIR})
include_directories(${OPENVDB_MINGW_BUILD_DIR})
include_directories(${OPENVDB_MINGW_BUILD_DIR}/boost)
include_directories(${OPENVDB_MINGW_BUILD_DIR}/ilmbase/Half)
add_definitions(-DNO_PCL_SUPPORT)
add_definitions(-DOPENVDB_OPENEXR_STATICLIB)
add_definitions(-DHALF_EXPORTS)
add_definitions(-UOPENEXR_DLL)
add_definitions(-DZLIB_WINAPI)
#add_definitions(-g -O2 -static ${OPENVDB_MINGW_BUILD_DIR}/libzip/lib/libz.a -Wl,-luuid -L"boost/stage/lib" -lboost_system-mgw48-mt-1_58 -lboost_iostreams-mgw48-mt-1_58)

add_library( nulibz STATIC IMPORTED )
set_target_properties( nulibz PROPERTIES IMPORTED_LOCATION ${OPENVDB_MINGW_BUILD_DIR}/libzip/lib/libz.a )

#remove_definitions()
link_directories(${OPENVDB_MINGW_BUILD_DIR}"/tbb/build/windows_ia32_gcc_mingw_release")
link_directories(${OPENVDB_MINGW_BUILD_DIR})
link_directories(${OPENVDB_MINGW_BUILD_DIR}"/libzip/lib")
add_library( libzip STATIC IMPORTED )
set_target_properties( libzip PROPERTIES IMPORTED_LOCATION ${OPENVDB_MINGW_BUILD_DIR}/libzip/lib/libz.a )
add_library( zlib SHARED IMPORTED )
#set_target_properties( zlib PROPERTIES IMPORTED_LOCATION  )

set_property(TARGET zlib PROPERTY IMPORTED_LOCATION ${OPENVDB_MINGW_BUILD_DIR}/zlib/zlib1.dll)

set_property(TARGET zlib PROPERTY IMPORTED_IMPLIB ${OPENVDB_MINGW_BUILD_DIR}/zlib/lib/zdll.lib)

add_library( zlib2 STATIC IMPORTED )
set_target_properties( zlib2 PROPERTIES IMPORTED_LOCATION ${OPENVDB_MINGW_BUILD_DIR}/zlib/zlib1.dll )
link_directories("${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib")

find_package(ZLIB)
message("zlibf" ${ZLIB_FOUND} " zlibrar " + ${ZLIB_LIBRARIES} )
if (ZLIB_FOUND)
    include_directories(${ZLIB_INCLUDE_DIRS})
    target_link_libraries(MyProg ${ZLIB_LIBRARIES})
endif()

endif(WIN32)

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

#file(COPY ${Datas} DESTINATION ${PROJECT_BINARY_DIR})
#file(COPY ${Resources} DESTINATION ${PROJECT_BINARY_DIR})

add_executable(${PROJECT_NAME}  ${SRC_LIST}
                                ${UPNS_SOURCES}
                                ${UPNS_VIS_RESOURCES_RCC}
                                ${FragmentShader}
                                ${VertexShader}
                                ${TesControlShader}
                                ${TesEvalShader}
                                ${GeometryShader}
                                ${GlslIncludeShader})

add_custom_target(resourceFilesViz ALL ${CMAKE_COMMAND} -E echo "nop" SOURCES ${Datas_All})

if(WIN32)
target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick ${Boost_LIBRARIES} uuid ${libzip} ${PCL_LIBRARIES} ${OPENVDB_MINGW_BUILD_DIR}/libopenvdb.a C:/MinGW/bin/zlib1.dll ${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib/libboost_system-mgw49-mt-1_58.a ${OPENVDB_MINGW_BUILD_DIR}/boost/stage/lib/libboost_iostreams-mgw49-mt-1_58.a nulibz Half tbb )
else(WIN32)
target_link_libraries(${PROJECT_NAME} Qt5::Qml Qt5::Widgets Qt5::Quick ${Boost_LIBRARIES} uuid ${libzip} ${PCL_LIBRARIES} openvdb Half tbb )
endif(WIN32)

target_link_libraries(${PROJECT_NAME} upns_mapmanager)

file(GLOB_RECURSE UPNS_QMLFILES ${PROJECT_SOURCE_DIR}/*.qml ${PROJECT_SOURCE_DIR}/*.js ${PROJECT_SOURCE_DIR}/*/qmldir)

add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add qml files to project" SOURCES ${UPNS_QMLFILES})

add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ln -sfT ${UPNS_ROOT_DIR}/data ${CMAKE_BINARY_DIR}/visualization/data
)

add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ln -sfT ${UPNS_ROOT_DIR}/resources ${CMAKE_BINARY_DIR}/visualization/resources
)

add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ln -sfT ${UPNS_ROOT_DIR}/logging.properties ${CMAKE_BINARY_DIR}/visualization/logging.properties
)

endif(USE_PCL)
