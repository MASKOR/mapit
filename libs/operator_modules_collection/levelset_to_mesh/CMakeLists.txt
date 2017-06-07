project(levelset_to_mesh)
cmake_minimum_required(VERSION 2.8)

set(OPERATOR_VERSION 1)

aux_source_directory(. SRC_LIST)

add_definitions(-DOPERATOR_NAME="${PROJECT_NAME}")

add_definitions(-DOPERATOR_VERSION=${OPERATOR_VERSION})

file(GLOB_RECURSE UPNS_OPERATOR_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

set(CMAKE_AUTOMOC ON)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

find_package(OpenVDB REQUIRED)
find_package(TBB REQUIRED)

add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${UPNS_OPERATOR_SOURCES} )
target_link_libraries( ${PROJECT_NAME} upns_interface upns_core layertype_asset layertype_openvdb)
target_link_libraries( ${PROJECT_NAME} ${OPENVDB_LIBRARIES} ${TBB_LIBRARIES} ${HALF_LIBRARIES})

qt5_use_modules(${PROJECT_NAME} Core Gui)

SET_TARGET_PROPERTIES(
    ${PROJECT_NAME}
  PROPERTIES
    VERSION ${OPERATOR_VERSION}
    SOVERSION ${OPERATOR_VERSION}
)

if (CMAKE_HOST_UNIX)
    set(FILENAME_PREFIX "lib")
    set(FILENAME_POSTFIX ".so")
else (CMAKE_HOST_UNIX)
    set(FILENAME_PREFIX "")
    set(FILENAME_POSTFIX ".dll")
endif (CMAKE_HOST_UNIX)

if(UNIX)
    # tell CMake Prefix is "lib" otherwise it can not be queried
    set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib")
endif(UNIX)
get_target_property(OLD_PREFIX ${PROJECT_NAME} PREFIX)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX ${OLD_PREFIX}${UPNS_INSTALL_OPERATORS})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
