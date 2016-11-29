project(networking_node)
cmake_minimum_required(VERSION 2.8)

find_package(Protobuf)
include_directories(${PROTOBUF_INCLUDE_DIRS})

#zeromq
find_package(ZMQ REQUIRED)
include_directories(${ZMQ_INCLUDE_DIR})

#cppzeromq
find_package(CPPZMQ REQUIRED)
include_directories(${CPPZMQ_INCLUDE_DIR})

file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h ${PROJECT_SOURCE_DIR}/include/*.h )

add_library(${PROJECT_NAME} ${UPNS_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC  ${PROJECT_SOURCE_DIR}/include
                                           PRIVATE ${PROJECT_SOURCE_DIR}/src )

target_link_libraries(${PROJECT_NAME} upns_mapmanager ${ZMQ_LIBRARIES} ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES} ${LOG4CPLUS_LIBRARIES})

if(NOT WIN32)
    target_link_libraries(${PROJECT_NAME} dl)
endif(NOT WIN32)

if(NOT WIN32)
  target_link_libraries(${PROJECT_NAME} uuid ${libzip})
endif(NOT WIN32)
