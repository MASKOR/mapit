project(upns_interface_cpp)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${UPNS_INTERFACE_INCLUDE})

file(GLOB_RECURSE UPNS_INTERFACE_CPP_SOURCES ${PROJECT_SOURCE_DIR}/include/*.h)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${PROJECT_SOURCE_DIR}/include/)

#shows files in qtcreator
add_custom_target(${PROJECT_NAME}_NOP ALL ${CMAKE_COMMAND} -E echo "show files in qtcreator" SOURCES ${UPNS_INTERFACE_CPP_SOURCES})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${CMAKE_LIBRARY_ARCHITECTURE})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${UPNS_INSTALL_INCLUDE} )
