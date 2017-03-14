project(layertype_asset)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

find_package(Protobuf)
include_directories(${PROTOBUF_INCLUDE_DIRS})

#find_package(assimp REQUIRED)

file(GLOB_RECURSE TINYPLY_SOURCES ${PROJECT_SOURCE_DIR}/tinyply/source/tinyply.cpp ${PROJECT_SOURCE_DIR}/tinyply/source/tinyply.h)

file(GLOB_RECURSE LAYERTYPE_ASSET_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h ${PROJECT_SOURCE_DIR}/include/*.h)

add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${TINYPLY_SOURCES} ${LAYERTYPE_ASSET_SOURCES})

target_link_libraries(${PROJECT_NAME} upns_interface upns_interface_cpp)# ${assimp_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC  ${PROJECT_SOURCE_DIR}/include/
                                                   ${PROJECT_SOURCE_DIR}/tinyply/source
                                           PRIVATE ${PROJECT_SOURCE_DIR}/src/
                                                   ${PROJECT_SOURCE_DIR}/../../mapmanager/include/
                                                   ${PROTOBUF_INCLUDE_DIRS})

target_compile_definitions(${PROJECT_NAME} PRIVATE PROJECT_NAME="${PROJECT_NAME}"
                                           PUBLIC  AssetEntitydata_TYPENAME="${PROJECT_NAME}")

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${UPNS_INSTALL_LAYERTYPES})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${UPNS_INSTALL_INCLUDE}/${UPNS_INSTALL_LAYERTYPES} )
