project(layertype_asset)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
#find_package(PCL REQUIRED COMPONENTS common io apps filters kdtree search features surface keypoints registration)


string( TOLOWER "${CMAKE_BUILD_TYPE}" LOWER_CMAKE_BUILD_TYPE )
if (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
    set(BOOST_LIBRARIES_ADDITION2 "${BOOST_LIB_SERIAL_DEBUG2}" "${BOOST_LIB_SERIAL_DEBUG3}")
else (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
    set(BOOST_LIBRARIES_ADDITION2 "${BOOST_LIB_SERIAL_RELEASE2}" "${BOOST_LIB_SERIAL_RELEASE3}")
endif (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")

include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${PROJECT_SOURCE_DIR}/../../mapmanager/include/)
include_directories(${PROJECT_SOURCE_DIR}/../../../externals/log4cplus/include/)
if(WIN32)
include_directories(${PROJECT_SOURCE_DIR}/../../../externals/windows/include/)
endif(WIN32)

use_protobuf()

file(GLOB_RECURSE LAYERTYPE_ASSET_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/src/*.h
                                                ${PROJECT_SOURCE_DIR}/include/*.h )

add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${LAYERTYPE_ASSET_SOURCES} ${PCL_CUSTOM_SRC} ${BOOST_LIBRARIES_ADDITION2} )
target_link_libraries( ${PROJECT_NAME} ${PCL_LIBRARIES} ${PCL_CUSTOM_LIB} )

#target_link_libraries(${PROJECT_NAME} Qt5::Core )

