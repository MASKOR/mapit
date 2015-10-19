project(upns)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

# c++11 for grpc dependencies
add_definitions(-std=c++11)

# add custom cmake scripts. i.e. FindProcessorCount...
set(CMAKE_MODULE_PATH
    ${PROJECT_SOURCE_DIR}/cmake
    ${CMAKE_MODULE_PATH}
)

#set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)

set(UPNS_QTVERSION)
include(FindProcessorCount)
message(STATUS "used number of processor cores: " ${PROCESSOR_COUNT})

file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)


set(DataDir ${PROJECT_SOURCE_DIR}/data)
file(GLOB Datas ${DataDir}/*)
file(GLOB_RECURSE Datas_All ${DataDir}/*)

include_directories( ${CMAKE_BINARY_DIR} ) # for proto headers

add_subdirectory(mapmanager)
add_subdirectory(operator_modules)
add_subdirectory(test)
add_subdirectory(upns_interface)
add_subdirectory(interface_grpc)
add_subdirectory(layertypes)
if(WITH_VISUALIZATION)
add_subdirectory(visualization)
endif(WITH_VISUALIZATION)

# make data visible in qtcreator
add_custom_target(resourceFiles ALL ${CMAKE_COMMAND} -E echo "add datas to qt project. nop" SOURCES ${Datas_All})

#target_link_libraries(${PROJECT_NAME} )

# make qml visible in qtcreator
file(GLOB_RECURSE UPNS_QMLFILES ${PROJECT_SOURCE_DIR}/*.qml ${PROJECT_SOURCE_DIR}/*/qmldir)

add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add qml files to project" SOURCES ${ECS_QMLFILES})
