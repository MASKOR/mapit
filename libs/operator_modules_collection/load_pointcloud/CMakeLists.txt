if(NOT DISABLE_PCL)
project(load_pointcloud)
cmake_minimum_required(VERSION 2.8)

set(OPERATOR_VERSION 1)

aux_source_directory(. SRC_LIST)

include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/../../mapmanager/include/)

add_definitions(-DOPERATOR_NAME="${PROJECT_NAME}")

add_definitions(-DOPERATOR_VERSION=${OPERATOR_VERSION})

include_directories(${UPNS_INTERFACE_INCLUDE})
find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB_RECURSE UPNS_OPERATOR_LOAD_POINTCLOUD_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

set(CMAKE_AUTOMOC ON)
find_package(Qt5Core ${UPNS_QTVERSION} REQUIRED)

#### Protobuf ####

set(PROTOFILE "messages/param.proto")
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# results in /bin/sh: --cpp_out: Kommando nicht gefunden.
#protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROJECT_SOURCE_DIR}/${PROTOFILE})
#set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES GENERATED TRUE)

############# selfmade but working proto generator ###########

foreach(FIL ${ProtoFiles})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)
#    add_custom_command(
#        OUTPUT "${FIL_WE}.grpc.pb.cc"
#             "${FIL_WE}.grpc.pb.h"
#        COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
#        ARGS --grpc_out=${PROTOBUF_OUTPUT_DIR} --proto_path ${CMAKE_CURRENT_SOURCE_DIR}/messages --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN} ${ABS_FIL}
#        DEPENDS ${ABS_FIL}
#        COMMENT "Running grpc protocol buffer compiler on ${FIL}"
#        VERBATIM )

    add_custom_command(
        OUTPUT "${FIL_WE}.pb.cc"
               "${FIL_WE}.pb.h"
        COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS --cpp_out ${PROTOBUF_OUTPUT_DIR} --proto_path ${CMAKE_CURRENT_SOURCE_DIR}/messages ${ABS_FIL}
        DEPENDS ${ABS_FIL}
        COMMENT "Running C++ protocol buffer compiler on ${FIL}"
        VERBATIM )


#    list(APPEND MESSAGES_SRCS "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.grpc.pb.cc")
#    list(APPEND MESSAGES_HDRS "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.grpc.pb.h")

    list(APPEND PROTO_SRCS "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.pb.cc")
    list(APPEND PROTO_HDRS "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.pb.h")

endforeach()

set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES GENERATED TRUE)

##############################################

add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${UPNS_OPERATOR_LOAD_POINTCLOUD_SOURCES} ${PROTO_SRCS} ${PROTO_HDRS} )
target_link_libraries( ${PROJECT_NAME} ${PCL_LIBRARIES} ${PROTOBUF_LIBRARIES} upns_interface upns_mapmanager Qt5::Core)

#process_protobuf_post()

SET_TARGET_PROPERTIES(
    ${PROJECT_NAME}
  PROPERTIES
    VERSION ${OPERATOR_VERSION}
    SOVERSION ${OPERATOR_VERSION}
)

#get_property(lib_location TARGET ${PROJECT_NAME} PROPERTY LOCATION)

if (CMAKE_HOST_UNIX)
    set(FILENAME_PREFIX "lib")
    set(FILENAME_POSTFIX ".so")
else (CMAKE_HOST_UNIX)
    set(FILENAME_PREFIX "")
    set(FILENAME_POSTFIX ".dll")
endif (CMAKE_HOST_UNIX)

#set(LIB_FILENAME ${FILENAME_PREFIX}${PROJECT_NAME}${CMAKE_DEBUG_POSTFIX}${FILENAME_POSTFIX})
#set(LIB_FILENAME_VERSION ${FILENAME_PREFIX}${PROJECT_NAME}${CMAKE_DEBUG_POSTFIX}${FILENAME_POSTFIX}.${OPERATOR_VERSION})

#add_custom_command(
#    TARGET ${PROJECT_NAME} POST_BUILD
#    COMMAND ln -sf ${LIB_FILENAME} ${LIB_FILENAME_VERSION} WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH}
#    COMMAND ln -sf ${LIB_FILENAME} ${LIB_FILENAME_VERSION} WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH}
#)
endif(NOT DISABLE_PCL)
