project(layertype_asset)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

find_package(Protobuf)
include_directories(${PROTOBUF_INCLUDE_DIRS})

find_package(assimp REQUIRED)

file(GLOB_RECURSE LAYERTYPE_OPENVDB_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h ${PROJECT_SOURCE_DIR}/include/*.h)

add_library(${PROJECT_NAME} ${SRC_LIST} ${LAYERTYPE_OPENVDB_SOURCES})

target_link_libraries(${PROJECT_NAME} ${OPENVDB_LIBRARIES} upns_interface upns_interface_cpp)

target_include_directories(${PROJECT_NAME} PUBLIC  ${PROJECT_SOURCE_DIR}/include/
                                           PRIVATE ${PROJECT_SOURCE_DIR}/src/
                                                   ${PROJECT_SOURCE_DIR}/../../mapmanager/include/
                                                   ${PROTOBUF_INCLUDE_DIRS})
