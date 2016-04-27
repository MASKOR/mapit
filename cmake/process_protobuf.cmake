#Do not use! Protobuf commands must be in the same file as the project. (sometimes cmake makes me angry)

#macro(process_protobuf FOLDER)
#    find_package(Protobuf REQUIRED)
#    include_directories(${PROTOBUF_INCLUDE_DIRS})
#    include_directories(${CMAKE_CURRENT_BINARY_DIR})
#    protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROJECT_SOURCE_DIR}/${FOLDER}/*.proto)
#    set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES GENERATED TRUE)

#    set(MESSAGES_SRCS ${PROTO_SRCS} PARENT_SCOPE)
#    set(MESSAGES_HDRS ${PROTO_HDRS} PARENT_SCOPE)
##    file(GLOB ProtoFiles ${PROJECT_SOURCE_DIR}/${FOLDER}/*.proto)

##    #set(PROTOBUF_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/generated)
##    set(PROTOBUF_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})

##    set(PROTOBUF_FILES_INCLUDE ${PROTOBUF_OUTPUT_DIR})

##    #find_package(Protobuf REQUIRED)
##    include_directories(${PROTOBUF_INCLUDE_DIRS})
##    include_directories(${PROTOBUF_FILES_INCLUDE})

##    ######## Protobuf + gRpc
##    #protobuf_generate_cpp(MESSAGES_SRCS MESSAGES_HDRS ${ProtoFiles})

##    #use_grpc()
##    #use_protobuf()

##    #protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS params.proto)
##    foreach(FIL ${ProtoFiles})
##        get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
##        get_filename_component(FIL_WE ${FIL} NAME_WE)
##    #    add_custom_command(
##    #        OUTPUT "${FIL_WE}.grpc.pb.cc"
##    #             "${FIL_WE}.grpc.pb.h"
##    #        COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
##    #        ARGS --grpc_out=${PROTOBUF_OUTPUT_DIR} --proto_path ${CMAKE_CURRENT_SOURCE_DIR}/messages --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN} ${ABS_FIL}
##    #        DEPENDS ${ABS_FIL}
##    #        COMMENT "Running grpc protocol buffer compiler on ${FIL}"
##    #        VERBATIM )

##        add_custom_command(
##            OUTPUT "${FIL_WE}.pb.cc"
##                   "${FIL_WE}.pb.h"
##            COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
##            ARGS --cpp_out ${PROTOBUF_OUTPUT_DIR} --proto_path ${CMAKE_CURRENT_SOURCE_DIR}/${FOLDER} ${ABS_FIL}
##            DEPENDS ${ABS_FIL}
##            COMMENT "Running C++ protocol buffer compiler on ${FIL}"
##            VERBATIM )


##    #    list(APPEND MESSAGES_SRCS "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.grpc.pb.cc")
##    #    list(APPEND MESSAGES_HDRS "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.grpc.pb.h")

##        list(APPEND MESSAGES_SRCS "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.pb.cc")
##        list(APPEND MESSAGES_HDRS "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.pb.h")
##        message("${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.pb.cc")
##        message("${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.pb.h")
##        message("${CMAKE_CURRENT_SOURCE_DIR}/${FOLDER}.gggggg.cc")
##        message("${PROJECT_SOURCE_DIR}/${FOLDER}/*.proto.gggggg.h")
##        message("${PROTOBUF_PROTOC_EXECUTABLE}.fffffffff")

##    endforeach()

#    set_source_files_properties(${MESSAGES_SRCS} ${MESSAGES_HDRS} PROPERTIES GENERATED TRUE)
#    #set(MESSAGES_SRCS ${MESSAGES_SRCS} PARENT_SCOPE)
#    #set(MESSAGES_HDRS ${MESSAGES_HDRS} PARENT_SCOPE)


#    ########
#    include_directories(${MESSAGES_HDRS})

#    #Note: On Win32 this can not be a 'shared' library as it would create a dll without any exported symbols! Thus, make it a static linked library
#    #add_library(${PROJECT_NAME} ${MESSAGES_SRCS} ${MESSAGES_HDRS})


#    # make proto visible in qtcreator
#    file(GLOB_RECURSE UPNS_PROTOFILES ${PROJECT_SOURCE_DIR}/messages/*.proto)
#    add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add proto files to project" SOURCES ${UPNS_PROTOFILES})

#endmacro(process_protobuf)

#macro(process_protobuf_post)
#    target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})
#    set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)
#    # all projects depending on this will include PROTOBUF_OUTPUT_DIR
#    target_include_directories(${PROJECT_NAME} PUBLIC ${PROTOBUF_OUTPUT_DIR})
#endmacro(process_protobuf_post)
