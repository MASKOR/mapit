project(upns_mapmanager)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

set(CMAKE_AUTOMOC ON)

custom_set_vars_leveldb()
#use_grpc()
#use_protobuf()

find_package(Qt5Core REQUIRED)

#FIND_PACKAGE( Boost COMPONENTS iostreams serialization REQUIRED )

#include_directories(${Boost_INCLUDE_DIRS})

link_directories(${YAML_CPP_LIB_DIR})

#find_package(Qt5 COMPONENTS DateTime REQUIRED)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${LEVELDB_INCLUDE_DIRECTORY})
include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${UPNS_INTERFACE_INCLUDE})
include_directories(${YAML_CPP_INCLUDE})

file(GLOB_RECURSE UPNS_MM_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h ${PROJECT_SOURCE_DIR}/include/*.h)

add_library(${PROJECT_NAME} ${SRC_LIST} ${UPNS_MM_SOURCES} )
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)
#Note: Windows (and also linux in some cases?):
# PCL was linked statically against boost. PCL is linkes statically to layertype.
# Layertype has nothing to do with mapmanager. (linked dynmic and at runtime)
# Somehow boost dependency comes in here. I guess a static library wants it, thus the executable/dynamic-library must have it.
#TODO: what is dl lib?
#target_link_libraries(${PROJECT_NAME} dl Qt5::Core upns_interface yaml-cpp ${LEVELDB_LIB} log4cplus ssl crypto)
#target_link_libraries(${PROJECT_NAME} Qt5::Core upns_interface yaml-cpp ${LEVELDB_LIB} log4cplus ssl crypto)
if(WIN32)
  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} Qt5::Core upns_interface ${YAML_CPP_LIBRARIES} ${LEVELDB_LIBRARIES} ${LOG4CPLUS_LIBRARIES})
else(WIN32)
  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} Qt5::Core upns_interface ${YAML_CPP_LIBRARIES} dl ${LEVELDB_LIBRARIES} ${LOG4CPLUS_LIBRARIES})
endif(WIN32)
#    E:/devel/boost_1_55_0/lib64-msvc-12.0/libboost_filesystem-vc120-mt-gd-1_55.lib
#    E:/devel/boost_1_55_0/lib64-msvc-12.0/libboost_system-vc120-mt-gd-1_55.lib
#    E:/devel/boost_1_55_0/lib64-msvc-12.0/libboost_date_time-vc120-mt-gd-1_55.lib)

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include/)
