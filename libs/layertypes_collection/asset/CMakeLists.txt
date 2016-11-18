project(layertype_asset)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
#find_package(PCL REQUIRED COMPONENTS common io apps filters kdtree search features surface keypoints registration)


#string( TOLOWER "${CMAKE_BUILD_TYPE}" LOWER_CMAKE_BUILD_TYPE )
#if (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
#else (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
#endif (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")

include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${PROJECT_SOURCE_DIR}/../../mapmanager/include/)
include_directories(${PROJECT_SOURCE_DIR}/../../../externals/log4cplus/include/)
if(WIN32)
include_directories(${PROJECT_SOURCE_DIR}/../../../externals/windows/include/)
endif(WIN32)

find_package(Protobuf)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})

file(GLOB_RECURSE LAYERTYPE_ASSET_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/src/*.h
                                                ${PROJECT_SOURCE_DIR}/include/*.h )

add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${LAYERTYPE_ASSET_SOURCES}  )
target_link_libraries( ${PROJECT_NAME} ${PCL_LIBRARIES} )

#custom_target_use_boost(${subdir})
find_package(Boost COMPONENTS serialization)
target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES})

