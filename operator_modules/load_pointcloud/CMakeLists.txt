project(upns_operator_load_pointloud)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/../../mapmanager/include/)

file(GLOB_RECURSE UPNS_OPERATOR_VOXELFILTER_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

add_library(${PROJECT_NAME} ${SRC_LIST} ${UPNS_OPERATOR_VOXELFILTER_SOURCES} )

#target_link_libraries(${PROJECT_NAME} Qt5::Core )
