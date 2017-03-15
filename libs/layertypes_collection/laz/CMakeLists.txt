project(layertype_laz)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

find_package(Protobuf)
find_package(Boost 1.42 COMPONENTS program_options thread system iostreams filesystem REQUIRED)

if(WITH_LOG4CPLUS)
  include_directories(${PROJECT_SOURCE_DIR}/../../../externals/log4cplus/include/)
endif(WITH_LOG4CPLUS)
if(WIN32)
  include_directories(${PROJECT_SOURCE_DIR}/../../../externals/windows/include/)
endif(WIN32)

add_definitions(-DPROJECT_NAME="${PROJECT_NAME}")

file(GLOB_RECURSE LAYERTYPE_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/src/*.h
                                                #${PROJECT_SOURCE_DIR}/laszip-src-2.2.0/src/*.hpp
                                                #${PROJECT_SOURCE_DIR}/laszip-src-2.2.0/src/*.cpp
                                                #${PROJECT_SOURCE_DIR}/laszip-src-2.2.0/include/*.hpp
                                                ${PROJECT_SOURCE_DIR}/libLAS/include/*.hpp
                                                ${PROJECT_SOURCE_DIR}/libLAS/include/*.h
                                                ${PROJECT_SOURCE_DIR}/libLAS/src/*.hpp
                                                ${PROJECT_SOURCE_DIR}/libLAS/src/*.h
                                                ${PROJECT_SOURCE_DIR}/libLAS/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/include/*.h )

set(HAVE_GDAL FALSE CACHE BOOL "Enable GDAL for LibLAS")
set(HAVE_LIBGEOTIFF FALSE CACHE BOOL "Enable libGeoTIFF for LibLAS")

if(NOT HAVE_GDAL)
  file(GLOB_RECURSE GT_SRC ${PROJECT_SOURCE_DIR}/libLAS/src/gt_*.cpp
                           ${PROJECT_SOURCE_DIR}/libLAS/src/gt_*.h)
  list(REMOVE_ITEM LAYERTYPE_SOURCES ${GT_SRC})
endif(NOT HAVE_GDAL)

if(NOT HAVE_LIBGEOTIFF)
  file(GLOB_RECURSE GEO_SRC ${PROJECT_SOURCE_DIR}/libLAS/src/tifvsi.cpp)
  list(REMOVE_ITEM LAYERTYPE_SOURCES ${GEO_SRC})
endif(NOT HAVE_LIBGEOTIFF)


add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${LAYERTYPE_SOURCES})
target_link_libraries( ${PROJECT_NAME} upns_interface upns_interface_cpp ${BOOST_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC  ${PROJECT_SOURCE_DIR}/include/
                                                   #${PROJECT_SOURCE_DIR}/laszip-src-2.2.0/include/
                                                   ${PROJECT_SOURCE_DIR}/libLAS/include
                                           PRIVATE ${PROJECT_SOURCE_DIR}/src/
                                                   ${PROTOBUF_INCLUDE_DIRS}
                                                   ${Boost_INCLUDE_DIRS})

target_compile_definitions(${PROJECT_NAME} PRIVATE PROJECT_NAME="${PROJECT_NAME}"
                                           PUBLIC  LASzipEntitydata_TYPENAME="${PROJECT_NAME}")

if(UNIX)
    # tell CMake Prefix is "lib" otherwise it can not be queried
    set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib")
endif(UNIX)
get_target_property(OLD_PREFIX ${PROJECT_NAME} PREFIX)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX ${OLD_PREFIX}${UPNS_INSTALL_LAYERTYPES})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${UPNS_INSTALL_INCLUDE}/${UPNS_INSTALL_LAYERTYPES} )
