project(layertype_tf)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${PROJECT_SOURCE_DIR}/../../mapmanager/include/)
if(WITH_LOG4CPLUS)
  include_directories(${PROJECT_SOURCE_DIR}/../../../externals/log4cplus/include/)
endif(WITH_LOG4CPLUS)
if(WIN32)
include_directories(${PROJECT_SOURCE_DIR}/../../../externals/windows/include/)
endif(WIN32)

include(FindLog4cplus)

find_package(Protobuf)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})


# Eigen3  Will just not be found on windows
if(WIN32)
    find_package(Eigen3)
    set(EIGEN3_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/externals/eigen3) # contains Eigen/...
else(WIN32)
    find_package(Eigen3 REQUIRED)
endif(WIN32)

# TODO: If language could not be determined: There are Additional cmake variable for boost that mus be set. Fix this
file(GLOB_RECURSE LAYERTYPE_TF_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/src/*.h
                                                ${PROJECT_SOURCE_DIR}/include/*.h )
add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${LAYERTYPE_TF_SOURCES} )
target_link_libraries( ${PROJECT_NAME} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})

find_package(Boost COMPONENTS serialization system filesystem date_time)
target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES})
target_link_libraries(${PROJECT_NAME} upns_interface upns_interface_cpp ${LOG4CPLUS_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})


target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES} upns_interface upns_interface_cpp ${LOG4CPLUS_LIBRARIES})

target_compile_definitions(${PROJECT_NAME} PRIVATE PROJECT_NAME="${PROJECT_NAME}"
                                           PUBLIC  TfEntitydata_TYPENAME="${PROJECT_NAME}")

if(UNIX)
  # tell CMake Prefix is "lib" otherwise it can not be queried
  set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib")
endif(UNIX)
get_target_property(OLD_PREFIX ${PROJECT_NAME} PREFIX)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX ${OLD_PREFIX}${UPNS_INSTALL_LAYERTYPES})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${UPNS_INSTALL_INCLUDE}/${UPNS_INSTALL_LAYERTYPES} )
