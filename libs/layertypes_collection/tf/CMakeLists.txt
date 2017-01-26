project(layertype_tf)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${PROJECT_SOURCE_DIR}/../../mapmanager/include/)
include_directories(${PROJECT_SOURCE_DIR}/../../../externals/log4cplus/include/)
if(WIN32)
include_directories(${PROJECT_SOURCE_DIR}/../../../externals/windows/include/)
endif(WIN32)

include(FindLog4cplus)

find_package(Protobuf)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)

# TODO: If language could not be determined: There are Additional cmake variable for boost that mus be set. Fix this
file(GLOB_RECURSE LAYERTYPE_TF_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/src/*.h
                                                ${PROJECT_SOURCE_DIR}/include/*.h )
add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${LAYERTYPE_TF_SOURCES} )
target_link_libraries( ${PROJECT_NAME} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})

find_package(Boost COMPONENTS serialization system filesystem date_time)
target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})


target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES} upns_interface upns_interface_cpp ${LOG4CPLUS_LIBRARIES})

target_compile_definitions(${PROJECT_NAME} PRIVATE PROJECT_NAME="${PROJECT_NAME}"
                                           PUBLIC  TfEntitydata_TYPENAME="${PROJECT_NAME}")
