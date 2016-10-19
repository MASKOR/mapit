project(upns_mapmanager)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

############ yaml

include(FindYamlCpp)
include_directories(${YAML_CPP_INCLUDE})

link_directories(${YAML_CPP_LIB_DIR})

############# Protobuf

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

############ leveldb

if (WIN32)
    set(LEVELDB_LIBRARY_DIRECTORY "${PROJECT_SOURCE_DIR}/externals/windows/lib-msvc2013-md-64" CACHE FILEPATH "leveldb lib")
    set(LEVELDB_INCLUDE_DIRECTORY "${PROJECT_SOURCE_DIR}/externals/windows/include" CACHE FILEPATH "leveldb lib")
    set(LEVELDB_LIB_RELEASE "${LEVELDB_LIBRARY_DIRECTORY}/leveldb.lib" CACHE FILEPATH "leveldb lib")
    set(LEVELDB_LIB_DEBUG "${LEVELDB_LIBRARY_DIRECTORY}/leveldbd.lib" CACHE FILEPATH "leveldb lib")
else(WIN32)
    set(LEVELDB_LIBRARY_DIRECTORY "${PROJECT_SOURCE_DIR}/externals/leveldb/" CACHE FILEPATH "leveldb lib")
    set(LEVELDB_INCLUDE_DIRECTORY "${PROJECT_SOURCE_DIR}/externals/leveldb/include" CACHE FILEPATH "leveldb lib")
    set(LEVELDB_LIB_RELEASE "${LEVELDB_LIBRARY_DIRECTORY}/libleveldb.so" CACHE FILEPATH "leveldb lib")
    # Note: by default leveldb overwrites the release version of its lib when building leveldb in "debug". No "d" in the nameis there by default
    set(LEVELDB_LIB_DEBUG "${LEVELDB_LIBRARY_DIRECTORY}/libleveldb.so" CACHE FILEPATH "leveldb lib")
endif(WIN32)

set(LEVELDB_LIBRARIES debug "${LEVELDB_LIB_DEBUG}" optimized "${LEVELDB_LIB_RELEASE}")

include_directories(${LEVELDB_INCLUDE_DIRECTORY})




file(GLOB_RECURSE UPNS_MM_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h ${PROJECT_SOURCE_DIR}/include/*.h)

file(GLOB UPNS_ADDITIONAL_NOT_FOUND_FILES_TEMP ${PROJECT_SOURCE_DIR}/src/serialization/file_system/*.cpp
                                               ${PROJECT_SOURCE_DIR}/src/serialization/file_system/*.h
                                               ${PROJECT_SOURCE_DIR}/src/sha/*.cpp
                                               ${PROJECT_SOURCE_DIR}/src/sha/*.h )

add_library(${PROJECT_NAME} ${SRC_LIST} ${UPNS_MM_SOURCES} ${UPNS_ADDITIONAL_NOT_FOUND_FILES_TEMP} )
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

#Note: Windows (and also linux in some cases?):
# PCL was linked statically against boost. PCL is linkes statically to layertype.
# Layertype has nothing to do with mapmanager. (linked dynmic and at runtime)
# Somehow boost dependency comes in here. I guess a static library wants it, thus the executable/dynamic-library must have it.
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} upns_interface upns_interface_cpp ${YAML_CPP_LIBRARIES} ${LEVELDB_LIBRARIES} ${LOG4CPLUS_LIBRARIES})

if(NOT WIN32)
    target_link_libraries(${PROJECT_NAME} dl)
endif(NOT WIN32)
#    E:/devel/boost_1_55_0/lib64-msvc-12.0/libboost_filesystem-vc120-mt-gd-1_55.lib
#    E:/devel/boost_1_55_0/lib64-msvc-12.0/libboost_system-vc120-mt-gd-1_55.lib
#    E:/devel/boost_1_55_0/lib64-msvc-12.0/libboost_date_time-vc120-mt-gd-1_55.lib)

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include/
                                           PRIVATE ${PROJECT_SOURCE_DIR}/src/)
