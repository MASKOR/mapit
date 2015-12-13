
if(USE_PCL)
project(layertype_pointcloud2)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
#find_package(PCL REQUIRED COMPONENTS common io apps filters kdtree search features surface keypoints registration)
find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${PROJECT_SOURCE_DIR}/../../mapmanager/include/)

file(GLOB_RECURSE LAYERTYPE_POINTCLOUD2_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/src/*.h
                                                ${PROJECT_SOURCE_DIR}/include/*.h )

add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${LAYERTYPE_POINTCLOUD2_SOURCES} )
target_link_libraries( ${PROJECT_NAME} ${PCL_LIBRARIES} )

#target_link_libraries(${PROJECT_NAME} Qt5::Core )

endif(USE_PCL)