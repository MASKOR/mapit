if(NOT DISABLE_PCL)
project(normalestimation)
cmake_minimum_required(VERSION 2.8)

#SET(CMAKE_CXX_FLAGS "-std=c++98")
#set (CMAKE_CXX_STANDARD 98)

set(OPERATOR_VERSION 1)

aux_source_directory(. SRC_LIST)

include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/../../mapmanager/include/)

add_definitions(-DOPERATOR_NAME="${PROJECT_NAME}")

add_definitions(-DOPERATOR_VERSION=${OPERATOR_VERSION})

include_directories(${UPNS_INTERFACE_INCLUDE})

set(CMAKE_AUTOMOC ON)
find_package(Qt5Core ${UPNS_QTVERSION} REQUIRED)

custom_set_vars_boost()
#find_package(PCL 1.7 REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common io segmentation sample_consensus filters kdtree search)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(PCL_REALLY_ALL_LIBRARIES ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_SEARCH_LIBRARIES} ${PCL_KDTREE_LIBRARIES})

file(GLOB_RECURSE UPNS_OPERATOR_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)
add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${UPNS_OPERATOR_SOURCES} )
target_link_libraries(${PROJECT_NAME} ${PCL_REALLY_ALL_LIBRARIES} upns_interface upns_mapmanager)
#target_link_libraries(${PROJECT_NAME} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_SAMPLE_CONSENSUS_LINRARIES} ${PCL_FILTERS_LIBRARIES})

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
endif(NOT DISABLE_PCL)