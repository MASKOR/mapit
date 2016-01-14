
project(layertype_pointcloud2)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
#find_package(PCL REQUIRED COMPONENTS common io apps filters kdtree search features surface keypoints registration)

if(NOT (DISABLE_PCL))
    find_package(PCL REQUIRED COMPONENTS common io)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
else( NOT (DISABLE_PCL))
    #find_package(Eigen3 REQUIRED)
    #include_directories(${EIGEN3_INCLUDE_DIR})
    #set(PCL_CUSTOM_ROOT ${UPNS_ROOT_DIR}/externals/pcl-custom/include)
    #include_directories(${PCL_CUSTOM_ROOT})
    #FILE(GLOB PCL_CUSTOM_SRC ${PCL_CUSTOM_ROOT}/impl/* ${PCL_CUSTOM_ROOT}/common/impl/* ${PCL_CUSTOM_ROOT}/io/impl/*)
    #set(PCL_CUSTOM_LIB ${Boost_SYSTEM_LIBRARY} ${Boost_SERIALIZATION_LIBRARY} )
endif (NOT (DISABLE_PCL))


include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${PROJECT_SOURCE_DIR}/../../mapmanager/include/)
include_directories(${PROJECT_SOURCE_DIR}/../../../externals/log4cplus/include/)

use_protobuf()

file(GLOB_RECURSE LAYERTYPE_POINTCLOUD2_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/src/*.h
                                                ${PROJECT_SOURCE_DIR}/include/*.h )

add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${LAYERTYPE_POINTCLOUD2_SOURCES} ${PCL_CUSTOM_SRC} )
target_link_libraries( ${PROJECT_NAME} ${PCL_LIBRARIES} ${PCL_CUSTOM_LIB})

#target_link_libraries(${PROJECT_NAME} Qt5::Core )

