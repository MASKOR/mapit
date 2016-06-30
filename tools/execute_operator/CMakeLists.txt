project(execute_operator)
cmake_minimum_required(VERSION 2.8)

include(../cmake/custom_add_leveldb)

# add custom cmake scripts. i.e. FindProcessorCount...
set(CMAKE_MODULE_PATH
    ${PROJECT_SOURCE_DIR}/cmake
    ${CMAKE_MODULE_PATH}
)

find_package(Protobuf)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)

include_directories( ${UPNS_ROOT_DIR}/libs/mapmanager/include ) # TODO: introduce var.
include_directories(${UPNS_INTERFACE_INCLUDE})

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

include(FindProcessorCount)
message(STATUS "used number of processor cores: " ${PROCESSOR_COUNT})

custom_set_vars_leveldb()
include_directories(${LEVELDB_INCLUDE_DIRECTORY})


set(YAML_CPP_LIBRARIES debug ${YAML_CPP_LIBRARY_DEBUG} optimized ${YAML_CPP_LIBRARY_RELEASE})

link_directories(${YAML_CPP_LIB_DIR})
include_directories(${YAML_CPP_INCLUDE})
include_directories(${PROJECT_SOURCE_DIR}/src/)

# make proto visible in qtcreator
file(GLOB_RECURSE YAML_FILES ${PROJECT_SOURCE_DIR}/*.yaml)
add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add yaml files to project" SOURCES ${YAML_FILES})


file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

add_executable(${PROJECT_NAME} ${UPNS_SOURCES})

target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Widgets)

if(WIN32)
  target_link_libraries(${PROJECT_NAME} upns_mapmanager ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES} ${LOG4CPLUS_LIBRARIES})
else(WIN32)
  target_link_libraries(${PROJECT_NAME} upns_mapmanager ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES} ${LOG4CPLUS_LIBRARIES} uuid ${libzip})
endif(WIN32)

if(WIN32)
    set(YAML_CPP_LIBRARIES debug "${YAML_CPP_LIBRARY_DEBUG}" optimized "${YAML_CPP_LIBRARY_RELEASE}")
else(WIN32)
    set(YAML_CPP_LIBRARIES "${YAML_CPP_LIBRARY_DEBUG}" "${YAML_CPP_LIBRARY_RELEASE}")
endif(WIN32)

execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/data ${CMAKE_BINARY_DIR}/tools/${PROJECT_NAME}/data
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/tools/${PROJECT_NAME}/libs
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${PROJECT_SOURCE_DIR}/src/repo.yaml ${CMAKE_BINARY_DIR}/tools/${PROJECT_NAME}/repo.yaml
)

execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/data ${CMAKE_BINARY_DIR}/tools/bin/data
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
