project(upns_mapmanager)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

find_package(Boost COMPONENTS filesystem iostreams)

############# Protobuf

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

file(GLOB_RECURSE UPNS_MM_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp
                                  ${PROJECT_SOURCE_DIR}/src/*.h
                                  ${PROJECT_SOURCE_DIR}/src/serialization/file_system/*.cpp
                                  ${PROJECT_SOURCE_DIR}/src/serialization/file_system/*.h
                                  ${PROJECT_SOURCE_DIR}/include/*.h
                                  ${PROJECT_SOURCE_DIR}/src/sha/*.cpp
                                  ${PROJECT_SOURCE_DIR}/src/sha/*.h)

add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${UPNS_MM_SOURCES} ${UPNS_ADDITIONAL_NOT_FOUND_FILES_TEMP} )
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

#Note: Windows (and also linux in some cases?):
# PCL was linked statically against boost. PCL is linkes statically to layertype.
# Layertype has nothing to do with mapmanager. (linked dynmic and at runtime)
# Somehow boost dependency comes in here. I guess a static library wants it, thus the executable/dynamic-library must have it.
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} upns_interface upns_interface_cpp ${BOOST_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY})

if(WITH_LOG4CPLUS)
  target_link_libraries(${PROJECT_NAME} ${LOG4CPLUS_LIBRARIES})
endif(WITH_LOG4CPLUS)

if(NOT WIN32)
    target_link_libraries(${PROJECT_NAME} dl)
endif(NOT WIN32)
#    E:/devel/boost_1_55_0/lib64-msvc-12.0/libboost_filesystem-vc120-mt-gd-1_55.lib
#    E:/devel/boost_1_55_0/lib64-msvc-12.0/libboost_system-vc120-mt-gd-1_55.lib
#    E:/devel/boost_1_55_0/lib64-msvc-12.0/libboost_date_time-vc120-mt-gd-1_55.lib)

target_include_directories(${PROJECT_NAME} PUBLIC  ${PROJECT_SOURCE_DIR}/include/
                                           PRIVATE ${PROJECT_SOURCE_DIR}/src/
                                                   ${Boost_INCLUDE_DIRS})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${CMAKE_LIBRARY_ARCHITECTURE})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${UPNS_INSTALL_INCLUDE}/${PROJECT_NAME} )
