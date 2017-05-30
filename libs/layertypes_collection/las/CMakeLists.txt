project(layertype_las)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

find_package(Protobuf)
find_package(Boost COMPONENTS program_options thread system iostreams filesystem REQUIRED)

if(WITH_LOG4CPLUS)
  include_directories(${PROJECT_SOURCE_DIR}/../../../externals/log4cplus/include/)
endif(WITH_LOG4CPLUS)
if(WIN32)
  include_directories(${PROJECT_SOURCE_DIR}/../../../externals/windows/include/)
endif(WIN32)

add_definitions(-DPROJECT_NAME="${PROJECT_NAME}")

file(GLOB_RECURSE LAYERTYPE_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/src/*.h
                                                ${PROJECT_SOURCE_DIR}/libLAS/include/*.hpp
                                                ${PROJECT_SOURCE_DIR}/libLAS/include/*.h
                                                ${PROJECT_SOURCE_DIR}/libLAS/src/*.hpp
                                                ${PROJECT_SOURCE_DIR}/libLAS/src/*.h
                                                ${PROJECT_SOURCE_DIR}/libLAS/src/*.cpp
                                                ${PROJECT_SOURCE_DIR}/include/*.h )

set(HAVE_GDAL FALSE CACHE BOOL "Enable GDAL for LibLAS")
set(HAVE_LIBGEOTIFF FALSE CACHE BOOL "Enable libGeoTIFF for LibLAS")
set(HAVE_LASZIP TRUE CACHE BOOL "Enable Laszip for LibLAS")

if(NOT HAVE_GDAL)
  file(GLOB_RECURSE GT_SRC ${PROJECT_SOURCE_DIR}/libLAS/src/gt_*.cpp
                           ${PROJECT_SOURCE_DIR}/libLAS/src/gt_*.h)
  list(REMOVE_ITEM LAYERTYPE_SOURCES ${GT_SRC})
endif(NOT HAVE_GDAL)

if(NOT HAVE_LIBGEOTIFF)
  file(GLOB_RECURSE GEO_SRC ${PROJECT_SOURCE_DIR}/libLAS/src/tifvsi.cpp)
  list(REMOVE_ITEM LAYERTYPE_SOURCES ${GEO_SRC})
endif(NOT HAVE_LIBGEOTIFF)

if(HAVE_LASZIP)
  file(GLOB_RECURSE LASZIP_SRC ${PROJECT_SOURCE_DIR}/LASzip/src/* ${PROJECT_SOURCE_DIR}/LASzip/include/*)
endif(HAVE_LASZIP)

add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${LAYERTYPE_SOURCES} ${LASZIP_SRC})
target_link_libraries( ${PROJECT_NAME} upns_interface upns_interface_cpp ${BOOST_LIBRARIES} ${Boost_LIBRARIES})


target_include_directories(${PROJECT_NAME} PUBLIC  ${PROJECT_SOURCE_DIR}/include/
                                                   #${PROJECT_SOURCE_DIR}/laszip-src-2.2.0/include/
                                                   ${PROJECT_SOURCE_DIR}/libLAS/include
                                           PRIVATE ${PROJECT_SOURCE_DIR}/src/
                                                   ${PROTOBUF_INCLUDE_DIRS}
                                                   ${Boost_INCLUDE_DIRS})

target_compile_definitions(${PROJECT_NAME} PRIVATE PROJECT_NAME="${PROJECT_NAME}"
                                           PUBLIC  LASEntitydata_TYPENAME="${PROJECT_NAME}"
                                                   WITH_LAS)

if(HAVE_LASZIP)
   target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/LASzip/src/
                                                      ${PROJECT_SOURCE_DIR}/LASzip/include/
                                                      ${PROJECT_SOURCE_DIR}/LASzip/include/laszip)
   target_compile_definitions(${PROJECT_NAME} PUBLIC HAVE_LASZIP)
endif(HAVE_LASZIP)

if(UNIX)
    # tell CMake Prefix is "lib" otherwise it can not be queried
    set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib")
endif(UNIX)
get_target_property(OLD_PREFIX ${PROJECT_NAME} PREFIX)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX ${OLD_PREFIX}${UPNS_INSTALL_LAYERTYPES})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${CMAKE_LIBRARY_ARCHITECTURE})
