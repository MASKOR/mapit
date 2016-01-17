
# NOTE: "git submodule update --init --recursive" may be used to initialize grpc correctly

set(EXTERNALS_DIR ${UPNS_ROOT_DIR}/externals)

#todo: grpc will just not work correctly
macro(use_grpc)

set(GRPC_CPP_PLUGIN ${EXTERNALS_DIR}/grpc/bins/opt/grpc_cpp_plugin)

if( NOT EXISTS ${GRPC_CPP_PLUGIN} )
message("Build grpc. If this fails, you may need to update git repo with \"git submodule update --init --recursive\"")
execute_process(COMMAND make -j${PROCESSOR_COUNT} WORKING_DIRECTORY ${EXTERNALS_DIR}/grpc)
#execute_process(COMMAND sudo make install WORKING_DIRECTORY ${EXTERNALS_DIR}/grpc)
endif( NOT EXISTS ${GRPC_CPP_PLUGIN} )

endmacro(use_grpc)

macro(use_leveldb)

#if( NOT EXISTS ${EXTERNALS_DIR}/leveldb/libleveldb.so )
#execute_process(COMMAND make -j${PROCESSOR_COUNT} WORKING_DIRECTORY ${EXTERNALS_DIR}/leveldb)
#endif( NOT EXISTS ${EXTERNALS_DIR}/leveldb/libleveldb.so )
if (WIN32)
link_directories(${EXTERNALS_DIR}/windows/lib-msvc2013-md-64)
include_directories(${EXTERNALS_DIR}/windows/include)
else()
link_directories(${EXTERNALS_DIR}/leveldb)
include_directories(${EXTERNALS_DIR}/leveldb/include)
endif()

set(LEVELDB_LIB leveldb)

endmacro(use_leveldb)

macro(use_protobuf)

find_package(Protobuf)

INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})
if(NOT EXISTS ${PROTOBUF_INCLUDE_DIRS})
    #Note: if this hangs, protobuf may does not support multithreaded build ( -j 8 )
    #execute_process(COMMAND ./configure WORKING_DIRECTORY ${EXTERNALS_DIR}/protobuf)
    #execute_process(COMMAND make -j${PROCESSOR_COUNT} WORKING_DIRECTORY ${EXTERNALS_DIR}/protobuf)
    #execute_process(COMMAND sudo make install WORKING_DIRECTORY ${EXTERNALS_DIR}/protobuf)

endif(NOT EXISTS ${PROTOBUF_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})
#if(NOT EXISTS ${PROTOBUF_LIBRARIES})
#    link_directories(${PROTOBUF_LIBRARY_PATH})
#    set(PROTOBUF_LIBRARIES protobuf)
#endif(NOT EXISTS ${PROTOBUF_LIBRARIES})

endmacro(use_protobuf)
