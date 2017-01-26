project(layertype_laz)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

find_package(Protobuf)

include_directories(${PROJECT_SOURCE_DIR}/../../../externals/log4cplus/include/)
if(WIN32)
  include_directories(${PROJECT_SOURCE_DIR}/../../../externals/windows/include/)
endif(WIN32)

add_definitions(-DPROJECT_NAME="${PROJECT_NAME}")

file(GLOB_RECURSE LAYERTYPE_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/src/*.h
                                                ${PROJECT_SOURCE_DIR}/laszip-src-2.2.0/src/*.hpp
                                                ${PROJECT_SOURCE_DIR}/laszip-src-2.2.0/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/laszip-src-2.2.0/include/*.hpp
                                                ${PROJECT_SOURCE_DIR}/include/*.h )
add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${LAYERTYPE_SOURCES}  )
target_link_libraries( ${PROJECT_NAME} upns_interface upns_interface_cpp)

target_include_directories(${PROJECT_NAME} PUBLIC  ${PROJECT_SOURCE_DIR}/include/
                                                   ${PROJECT_SOURCE_DIR}/laszip-src-2.2.0/include/
                                           PRIVATE ${PROJECT_SOURCE_DIR}/src/
                                                   ${PROTOBUF_INCLUDE_DIRS})

target_compile_definitions(${PROJECT_NAME} PRIVATE PROJECT_NAME="${PROJECT_NAME}"
                                           PUBLIC  LASzipEntitydata_TYPENAME="${PROJECT_NAME}")
