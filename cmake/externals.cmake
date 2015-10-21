project(externals)
cmake_minimum_required(VERSION 2.8)

# NOTE: "git submodule update --init --recursive" may be used to initialize grpc correctly

set(EXTERNALS_DIR ${UPNS_ROOT_DIR}/externals)

#todo: grpc will just not work correctly
macro(use_grpc)

set(GRPC_CPP_PLUGIN ${EXTERNALS_DIR}/grpc/bins/opt/grpc_cpp_plugin)

if( NOT EXISTS ${GRPC_CPP_PLUGIN} )
message("Build grpc. If this fails, you may need to update git repo with \"git submodule update --init --recursive\"")
execute_process(COMMAND make -j${PROCESSOR_COUNT} WORKING_DIRECTORY ${EXTERNALS_DIR}/grpc)
#execute_process(COMMAND sudo make install WORKING_DIRECTORY ${EXTERNALS_DIR}/grpc)
endif()

endmacro(use_grpc)

macro(use_leveldb)

if( NOT EXISTS ${EXTERNALS_DIR}/leveldb/libleveldb.so )
execute_process(COMMAND make -j${PROCESSOR_COUNT} WORKING_DIRECTORY ${EXTERNALS_DIR}/leveldb)
endif()

link_directories(${EXTERNALS_DIR}/leveldb)
include_directories(${EXTERNALS_DIR}/leveldb/include)

set(LEVELDB_LIB leveldb)

endmacro(use_leveldb)

macro(use_protobuf)

find_package(Protobuf)
if(NOT EXISTS ${PROTOBUF_INCLUDE_DIRS})
execute_process(COMMAND ./configure WORKING_DIRECTORY ${EXTERNALS_DIR}/protobuf)
execute_process(COMMAND make -j${PROCESSOR_COUNT} WORKING_DIRECTORY ${EXTERNALS_DIR}/protobuf)
#execute_process(COMMAND sudo make install WORKING_DIRECTORY ${EXTERNALS_DIR}/protobuf)
endif(NOT ${PROTOBUF_INCLUDE_DIRS})

endmacro(use_protobuf)
