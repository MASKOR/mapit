if(NOT DISABLE_PCL)
project(updatemetadata)
cmake_minimum_required(VERSION 2.8)

set(OPERATOR_VERSION 1)

aux_source_directory(. SRC_LIST)

include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/../../mapmanager/include/)

add_definitions(-DOPERATOR_NAME="${PROJECT_NAME}")

add_definitions(-DOPERATOR_VERSION=${OPERATOR_VERSION})

include_directories(${UPNS_INTERFACE_INCLUDE})

find_package(Protobuf)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})

# Note: Using c++11 in combination with pcl voxelgrid (1.7) filter
# can lead problems resulting from to incompatible ABIs
# (
# solution:
# git clone https://github.com/PointCloudLibrary/pcl.git
# git checkout pcl-1.7.2
# add line 'SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")' to CMakeLists.txt
# mkdir build
# cd build
# cmake ..
# make
# sudo make install
find_package(PCL REQUIRED COMPONENTS common io filters)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#set(PCL_REALLY_ALL_LIBRARIES ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES})

file(GLOB_RECURSE UPNS_OPERATOR_LOAD_POINTCLOUD_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)
message("PCL COMMON AND IO" ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
message("PCL FILTER" ${PCL_FILTER_LIBRARIES})
message("PCL FILTERS" ${PCL_FILTERS_LIBRARIES})
message("PCL PCL_LIBRARIES" ${PCL_LIBRARIES})
add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${UPNS_OPERATOR_LOAD_POINTCLOUD_SOURCES} )
target_link_libraries( ${PROJECT_NAME} ${PCL_REALLY_ALL_LIBRARIES} upns_interface upns_mapmanager)

#get_property(lib_location TARGET ${PROJECT_NAME} PROPERTY LOCATION)
SET_TARGET_PROPERTIES(
    ${PROJECT_NAME}
  PROPERTIES
    VERSION ${OPERATOR_VERSION}
    SOVERSION ${OPERATOR_VERSION}
)

if (CMAKE_HOST_UNIX)
    set(FILENAME_PREFIX "lib")
    set(FILENAME_POSTFIX ".so")
else (CMAKE_HOST_UNIX)
    set(FILENAME_PREFIX "")
    set(FILENAME_POSTFIX ".dll")
endif (CMAKE_HOST_UNIX)

#set(LIB_FILENAME ${FILENAME_PREFIX}${PROJECT_NAME}${CMAKE_DEBUG_POSTFIX}${FILENAME_POSTFIX})
#set(LIB_FILENAME_VERSION ${FILENAME_PREFIX}${PROJECT_NAME}${CMAKE_DEBUG_POSTFIX}${FILENAME_POSTFIX}.${OPERATOR_VERSION})

#add_custom_command(
#    TARGET ${PROJECT_NAME} POST_BUILD
#    COMMAND ln -sf ${LIB_FILENAME} ${LIB_FILENAME_VERSION} WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH}
#    COMMAND ln -sf ${LIB_FILENAME} ${LIB_FILENAME_VERSION} WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH}
#)
endif(NOT DISABLE_PCL)
