project(log)
cmake_minimum_required(VERSION 2.8)

include(../cmake/custom_add_leveldb)

# add custom cmake scripts. i.e. FindProcessorCount...
set(CMAKE_MODULE_PATH
    ${PROJECT_SOURCE_DIR}/cmake
    ${CMAKE_MODULE_PATH}
)

use_protobuf()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)

INCLUDE_DIRECTORIES( UPNS_ROOT_DIR )

find_package(Qt5Core ${UPNS_QTVERSION} REQUIRED)
find_package(Qt5Widgets ${UPNS_QTVERSION} REQUIRED)

include(FindProcessorCount)
message(STATUS "used number of processor cores: " ${PROCESSOR_COUNT})

custom_set_vars_leveldb()
include_directories(${LEVELDB_INCLUDE_DIRECTORY})

custom_set_vars_yamlcpp()
include_directories(${YAML_CPP_INCLUDE})
include_directories(${PROJECT_SOURCE_DIR}/src/)

# make proto visible in qtcreator
file(GLOB_RECURSE YAML_FILES ${PROJECT_SOURCE_DIR}/*.yaml)
add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add yaml files to project" SOURCES ${YAML_FILES})


file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

add_executable(${PROJECT_NAME} ${UPNS_SOURCES})

if(WIN32)
  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES}  ${BOOST_LIBRARIES_ADDITION})
else(WIN32)
  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} uuid ${libzip} ${BOOST_LIBRARIES_ADDITION})
endif(WIN32)

target_link_libraries(${PROJECT_NAME} upns_mapmanager ${LEVELDB_LIBRARIES} ${YAML_CPP_LIBRARIES})

execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/tools/${PROJECT_NAME}/libs
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${PROJECT_SOURCE_DIR}/src/repo.yaml ${CMAKE_BINARY_DIR}/tools/${PROJECT_NAME}/repo.yaml
)

execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/tools/bin/libs
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${PROJECT_SOURCE_DIR}/src/repo.yaml ${CMAKE_BINARY_DIR}/tools/bin/repo.yaml
)
execute_process(
    COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/tools/${PROJECT_NAME}/${PROJECT_NAME} ${CMAKE_BINARY_DIR}/tools/bin/${PROJECT_NAME}
)
