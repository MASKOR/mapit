project(networking_node)
cmake_minimum_required(VERSION 2.8)

find_package(Protobuf)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)

#zeromq
find_package(ZMQ REQUIRED)
include_directories(${ZMQ_INCLUDE_DIR})

#cppzeromq
find_package(CPPZMQ REQUIRED)
include_directories(${CPPZMQ_INCLUDE_DIR})

set(YAML_CPP_LIBRARIES debug ${YAML_CPP_LIBRARY_DEBUG} optimized ${YAML_CPP_LIBRARY_RELEASE})

include_directories(${YAML_CPP_INCLUDE})
include_directories(${PROJECT_SOURCE_DIR}/src/)

# make proto visible in qtcreator
file(GLOB_RECURSE YAML_FILES ${PROJECT_SOURCE_DIR}/*.yaml)
add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add yaml files to project" SOURCES ${YAML_FILES})


file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h ${PROJECT_SOURCE_DIR}/include/*.h )


add_library(${PROJECT_NAME} ${UPNS_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC  ${PROJECT_SOURCE_DIR}/include
                                           PRIVATE ${PROJECT_SOURCE_DIR}/src )

target_link_libraries(${PROJECT_NAME} upns_mapmanager ${ZMQ_LIBRARIES} ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES} ${LOG4CPLUS_LIBRARIES})

if(NOT WIN32)
  target_link_libraries(${PROJECT_NAME} uuid ${libzip})
endif(NOT WIN32)

if(WIN32)
    set(YAML_CPP_LIBRARIES debug "${YAML_CPP_LIBRARY_DEBUG}" optimized "${YAML_CPP_LIBRARY_RELEASE}")
else(WIN32)
    set(YAML_CPP_LIBRARIES "${YAML_CPP_LIBRARY_DEBUG}" "${YAML_CPP_LIBRARY_RELEASE}")
endif(WIN32)

