project(layertype_octomap)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/../../mapmanager/include/)

file(GLOB_RECURSE LAYERTYPE_OCTOMAP_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${LAYERTYPE_OCTOMAP_SOURCES} )

target_compile_definitions(${PROJECT_NAME} PRIVATE PROJECT_NAME="${PROJECT_NAME}"
                                           PUBLIC  OctomapEntitydata_TYPENAME="${PROJECT_NAME}")

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${UPNS_INSTALL_LAYERTYPES})
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${UPNS_INSTALL_INCLUDE}/${UPNS_INSTALL_LAYERTYPES} )
