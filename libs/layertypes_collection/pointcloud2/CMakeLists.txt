project(layertype_pointcloud2)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
#find_package(PCL REQUIRED COMPONENTS common io apps filters kdtree search features surface keypoints registration)

set (CMAKE_CXX_STANDARD 11)

find_package(Boost COMPONENTS serialization system filesystem date_time)

find_package(Protobuf)
include_directories(${PROTOBUF_INCLUDE_DIRS})

find_package(PCL REQUIRED COMPONENTS common io)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CMAKE_SOURCE_DIR}/externals/log4cplus/include/)
if(WIN32)
  include_directories(${CMAKE_SOURCE_DIR}/externals/windows/include/)
endif(WIN32)

# find Eigen3  Will just not work on windows
if(WIN32)
    find_package(Eigen3)
    set(EIGEN3_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/externals/eigen3) # contains eigen3/Eigen/...
else(WIN32)
    find_package(Eigen3 REQUIRED)
endif(WIN32)

# TODO: If language could not be determined: There are Additional cmake variable for boost that must be set. Fix this
file(GLOB_RECURSE LAYERTYPE_POINTCLOUD2_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/src/*.h
                                                ${PROJECT_SOURCE_DIR}/include/*.h )
add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${LAYERTYPE_POINTCLOUD2_SOURCES}  )
target_link_libraries( ${PROJECT_NAME} ${BOOST_LIBRARIES} ${PCL_LIBRARIES} upns_interface upns_interface_cpp)

target_link_libraries(${PROJECT_NAME} )
target_include_directories(${PROJECT_NAME} PUBLIC  ${PCL_INCLUDE_DIRS}
                                                   ${PROJECT_SOURCE_DIR}/include/
                                                   ${EIGEN3_INCLUDE_DIRS}
                                           PRIVATE ${PROJECT_SOURCE_DIR}/src/
                                                   ${PROTOBUF_INCLUDE_DIRS})

