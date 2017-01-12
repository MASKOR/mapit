project(databasedump)
cmake_minimum_required(VERSION 2.8)

find_package(Protobuf)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})

set(YAML_CPP_LIBRARIES debug ${YAML_CPP_LIBRARY_DEBUG} optimized ${YAML_CPP_LIBRARY_RELEASE})

include_directories(${YAML_CPP_INCLUDE})
include_directories(${LEVELDB_INCLUDE_DIRECTORY})
include_directories(${PROJECT_SOURCE_DIR}/src/)

file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

add_executable(${PROJECT_NAME} ${UPNS_SOURCES})

if(WIN32)
  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
else(WIN32)
  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} uuid ${libzip})
endif(WIN32)

target_link_libraries(${PROJECT_NAME} upns_mapmanager ${LEVELDB_LIBRARIES})

execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/tools/${PROJECT_NAME}/libs
)

execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/tools/bin/libs
)
execute_process(
    COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/tools/${PROJECT_NAME}/${PROJECT_NAME} ${CMAKE_BINARY_DIR}/tools/bin/${PROJECT_NAME}
)
