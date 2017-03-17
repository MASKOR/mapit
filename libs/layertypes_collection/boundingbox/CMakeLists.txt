project(boundingbox)
cmake_minimum_required(VERSION 3.0.2)
aux_source_directory(. SRC_LIST)
#find_package(PCL REQUIRED COMPONENTS common io apps filters kdtree search features surface keypoints registration)

find_package(Boost COMPONENTS serialization system filesystem date_time)

find_package(Protobuf)
include_directories(${PROTOBUF_INCLUDE_DIRS})

find_package(PCL REQUIRED COMPONENTS common io)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(WITH_LOG4CPLUS)
  include_directories(${PROJECT_SOURCE_DIR}/../../../externals/log4cplus/include/)
endif(WITH_LOG4CPLUS)
if(WIN32)
  include_directories(${PROJECT_SOURCE_DIR}/../../../externals/windows/include/)
endif(WIN32)


# TODO: If language could not be determined: There are Additional cmake variable for boost that mus be set. Fix this
file(GLOB_RECURSE LAYERTYPE_POINTCLOUD2_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/src/*.h
                                                ${PROJECT_SOURCE_DIR}/include/*.h )
add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${LAYERTYPE_POINTCLOUD2_SOURCES}  )
target_link_libraries( ${PROJECT_NAME} ${BOOST_LIBRARIES} ${PCL_LIBRARIES} upns_interface upns_interface_cpp)

target_link_libraries(${PROJECT_NAME} )
target_include_directories(${PROJECT_NAME} PUBLIC  ${PCL_INCLUDE_DIRS}
                                                   ${PROJECT_SOURCE_DIR}/include/
                                           PRIVATE ${PROJECT_SOURCE_DIR}/src/
                                                   ${PROTOBUF_INCLUDE_DIRS})

target_compile_definitions(${PROJECT_NAME} PRIVATE PROJECT_NAME="${PROJECT_NAME}"
                                           PUBLIC  BoundingboxEntitydata_TYPENAME="${PROJECT_NAME}")

if(UNIX)
   # tell CMake Prefix is "lib" otherwise it can not be queried
   set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib")
endif(UNIX)
get_target_property(OLD_PREFIX ${PROJECT_NAME} PREFIX)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX ${OLD_PREFIX}${UPNS_INSTALL_LAYERTYPES})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${CMAKE_LIBRARY_ARCHITECTURE}/${UPNS_INSTALL_INCLUDE}/${UPNS_INSTALL_INCLUDE_LAYERTYPES} )
