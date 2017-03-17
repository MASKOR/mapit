project(layertype_openvdb)

set(WITH_OPENVDB TRUE CACHE BOOL "Enable OpenVDB Layers")

cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

find_package(Protobuf)
include_directories(${PROTOBUF_INCLUDE_DIRS})

find_package(OpenVDB REQUIRED)
find_package(TBB REQUIRED)

file(GLOB_RECURSE LAYERTYPE_OPENVDB_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h ${PROJECT_SOURCE_DIR}/include/*.h)

add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${LAYERTYPE_OPENVDB_SOURCES})

target_link_libraries(${PROJECT_NAME} ${OPENVDB_LIBRARIES} ${TBB_LIBRARIES} ${HALF_LIBRARIES} upns_interface upns_interface_cpp)

target_include_directories(${PROJECT_NAME} PUBLIC  ${PROJECT_SOURCE_DIR}/include/
                                           PRIVATE ${PROJECT_SOURCE_DIR}/src/
                                                   ${PROJECT_SOURCE_DIR}/../../mapmanager/include/
                                                   ${PROTOBUF_INCLUDE_DIRS})

target_compile_definitions(${PROJECT_NAME} PRIVATE PROJECT_NAME="${PROJECT_NAME}"
                                           PUBLIC  FloatGridEntitydata_TYPENAME="${PROJECT_NAME}")

if(UNIX)
  # tell CMake Prefix is "lib" otherwise it can not be queried
  set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib")
endif(UNIX)
get_target_property(OLD_PREFIX ${PROJECT_NAME} PREFIX)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX ${OLD_PREFIX}${UPNS_INSTALL_LAYERTYPES})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${CMAKE_LIBRARY_ARCHITECTURE} )
