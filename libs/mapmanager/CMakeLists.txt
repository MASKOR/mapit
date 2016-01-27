project(upns_mapmanager)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

set(CMAKE_AUTOMOC ON)


#set(BOOST_LIB_FILESYSTEM_DEBUG "libboost_filesystem-vc120-gd-1_56.lib" CACHE FILEPATH "")
#set(BOOST_LIB_FILESYSTEM_RELEASE "libboost_filesystem-vc120-1_56.lib" CACHE FILEPATH "")
#set(BOOST_LIB_SYSTEM_DEBUG "libboost_system-vc120-gd-1_55.lib" CACHE FILEPATH "")
#set(BOOST_LIB_SYSTEM_RELEASE "libboost_system-vc120-1_55.lib" CACHE FILEPATH "")
#set(BOOST_LIB_DATE_TIME_DEBUG "libboost_date_time-vc120-gd-1_55.lib" CACHE FILEPATH "")
#set(BOOST_LIB_DATE_TIME_RELEASE "libboost_date_time-vc120-1_55.lib" CACHE FILEPATH "")

# TODO: Introduce FindXXX Scripts streamline librar loading for upns
# Take into account linux and windows. Packet manager installations, "externals" folder installations
set(LEVELDB_LIB_RELEASE "${EXTERNALS_DIR}/leveldb/libleveldb.so" CACHE FILEPATH "leveldb lib")
set(LEVELDB_LIB_DEBUG "${EXTERNALS_DIR}/leveldb/libleveldbd.so" CACHE FILEPATH "leveldb lib")
#set(YAML_LIB_RELEASE "yaml-cpp" CACHE FILEPATH "yaml-cpp md lib")
#set(YAML_LIB_DEBUG "yaml-cppd" CACHE FILEPATH "yaml-cpp md lib")

string( TOLOWER "${CMAKE_BUILD_TYPE}" LOWER_CMAKE_BUILD_TYPE )
if (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
    set(LEVELDB_LIBRARIES ${LEVELDB_LIB_DEBUG})
    set(YAML_CPP_LIBRARIES ${YAML_LIB_DEBUG})
    #set(LOG4CPLUS_LIBRARIES ${LOG4CPLUS_LIBRARY_DEBUG}) #Note: I dont know why this must be done manually for log4cplus with its findscript (windows)
else (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
    set(LEVELDB_LIBRARIES ${LEVELDB_LIB_RELEASE})
    set(YAML_CPP_LIBRARIES ${YAML_LIB_RELEASE})
    #set(LOG4CPLUS_LIBRARIES ${LOG4CPLUS_LIBRARY_RELEASE})
endif (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
find_package(Qt5Core ${UPNS_QTVERSION} REQUIRED)

use_leveldb()
#use_grpc()
use_protobuf()

link_directories(${EXTERNALS_DIR}/leveldb)
link_directories(${YAML_CPP_LIB_DIR})


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
target_link_libraries(${PROJECT_NAME} Qt5::Core upns_interface ${YAML_CPP_LIBRARIES} ${LEVELDB_LIBRARIES} ${LOG4CPLUS_LIBRARIES})
#    E:/devel/boost_1_55_0/lib64-msvc-12.0/libboost_filesystem-vc120-mt-gd-1_55.lib
#    E:/devel/boost_1_55_0/lib64-msvc-12.0/libboost_system-vc120-mt-gd-1_55.lib
#    E:/devel/boost_1_55_0/lib64-msvc-12.0/libboost_date_time-vc120-mt-gd-1_55.lib)

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include/)
