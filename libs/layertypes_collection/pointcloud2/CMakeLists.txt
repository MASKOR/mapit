project(layertype_pointcloud2)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
#find_package(PCL REQUIRED COMPONENTS common io apps filters kdtree search features surface keypoints registration)

find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#link_directories(${BOOST_ADDITIONAL_LINK_DIRECTORY})
include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${PROJECT_SOURCE_DIR}/../../mapmanager/include/)
include_directories(${PROJECT_SOURCE_DIR}/../../../externals/log4cplus/include/)
if(WIN32)
include_directories(${PROJECT_SOURCE_DIR}/../../../externals/windows/include/)
endif(WIN32)

find_package(Protobuf)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})

# TODO: If language could not be determined: There are Additional cmake variable for boost that mus be set. Fix this
file(GLOB_RECURSE LAYERTYPE_POINTCLOUD2_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/src/*.h
                                                ${PROJECT_SOURCE_DIR}/include/*.h )
add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${LAYERTYPE_POINTCLOUD2_SOURCES}  )
target_link_libraries( ${PROJECT_NAME} ${PCL_LIBRARIES} )

find_package(Boost COMPONENTS serialization system filesystem date_time)
target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})

#target_link_libraries(${PROJECT_NAME} Qt5::Core )

