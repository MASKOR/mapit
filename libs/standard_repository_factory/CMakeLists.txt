project(standard_repository_factory)
cmake_minimum_required(VERSION 2.8)

find_package(Boost COMPONENTS program_options)

find_package(Protobuf)
include_directories(${PROTOBUF_INCLUDE_DIRS})

#zeromq
find_package(ZMQ REQUIRED)
include_directories(${ZMQ_INCLUDE_DIR})

#cppzeromq
find_package(CPPZMQ REQUIRED)
include_directories(${CPPZMQ_INCLUDE_DIR})

set(WITH_YAML ON CACHE BOOL "Enable Yaml for repository configuration")
if(WITH_YAML)
  #yamlcpp
  find_package(YamlCpp)
  include_directories(${YAMLCPP_INCLUDE_DIR})
endif(WITH_YAML)

file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h ${PROJECT_SOURCE_DIR}/include/*.h )

add_library(${PROJECT_NAME} ${UPNS_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC  ${PROJECT_SOURCE_DIR}/include
                                           PRIVATE ${PROJECT_SOURCE_DIR}/src )

target_link_libraries(${PROJECT_NAME} upns_mapmanager networking_node ${ZMQ_LIBRARIES} ${Boost_LIBRARIES} ${YAMLCPP_LIBRARY})

if(WITH_LOG4CPLUS)
  target_link_libraries(${PROJECT_NAME} ${LOG4CPLUS_LIBRARIES})
endif(WITH_LOG4CPLUS)

if(NOT WIN32)
    target_link_libraries(${PROJECT_NAME} dl)
endif(NOT WIN32)

if(NOT WIN32)
  target_link_libraries(${PROJECT_NAME} uuid ${libzip})
endif(NOT WIN32)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${CMAKE_LIBRARY_ARCHITECTURE})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${CMAKE_LIBRARY_ARCHITECTURE}/${UPNS_INSTALL_INCLUDE}/${PROJECT_NAME} )
