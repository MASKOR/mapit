if(NOT DISABLE_PCL)
project(surface_reconstruction)
cmake_minimum_required(VERSION 2.8)

set(OPERATOR_VERSION 1)

find_package(mapit REQUIRED)

aux_source_directory(. SRC_LIST)

include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/../../mapmanager/include/)

add_definitions(-DOPERATOR_NAME="${PROJECT_NAME}")

add_definitions(-DOPERATOR_VERSION=${OPERATOR_VERSION})

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

set(CMAKE_AUTOMOC ON)

#custom_set_vars_boost()
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters surface segmentation octree features)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(PCL_REALLY_ALL_LIBRARIES ${PROTOBUF_LIBRARIES} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES})


find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

file(GLOB_RECURSE UPNS_OPERATOR_VOXELGRIDFILTER_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)
add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${UPNS_OPERATOR_VOXELGRIDFILTER_SOURCES})

target_link_libraries( ${PROJECT_NAME} ${PCL_REALLY_ALL_LIBRARIES} ${mapit_interface} ${mapit_core} layertype_pointcloud2 layertype_asset)

#if(TARGET json11)
    target_link_libraries( ${PROJECT_NAME} json11)
#else(TARGET json11)
#    target_include_directories(${PROJECT_NAME} PUBLIC ${JSON11_INCLUDE_DIR})
#    target_link_libraries( ${PROJECT_NAME} ${JSON11_LIBRARY})
#endif(TARGET json11)

SET_TARGET_PROPERTIES(
    ${PROJECT_NAME}
  PROPERTIES
    VERSION ${OPERATOR_VERSION}
    SOVERSION ${OPERATOR_VERSION}
)

#get_property(lib_location TARGET ${PROJECT_NAME} PROPERTY LOCATION)

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
if(UNIX)
    # tell CMake Prefix is "lib" otherwise it can not be queried
    set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib")
endif(UNIX)
get_target_property(OLD_PREFIX ${PROJECT_NAME} PREFIX)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX ${OLD_PREFIX}${UPNS_INSTALL_OPERATORS})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

endif(NOT DISABLE_PCL)