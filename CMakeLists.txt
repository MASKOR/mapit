project(upns)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

#if(UNIX)
#set (CMAKE_PREFIX_PATH ${QTDIR}"/gcc_64")
#endif(UNIX)

# add custom cmake scripts. i.e. FindProcessorCount...
set(CMAKE_MODULE_PATH
    ${PROJECT_SOURCE_DIR}/cmake
    ${CMAKE_MODULE_PATH}
)

#set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)

set(UPNS_QTVERSION 5.5)

include(FindProcessorCount)
message(STATUS "used number of processor cores: " ${PROCESSOR_COUNT})

file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)


set(DataDir ${PROJECT_SOURCE_DIR}/data)
file(GLOB Datas ${DataDir}/*)
file(GLOB_RECURSE Datas_All ${DataDir}/*)

add_subdirectory(mapmanager)
add_subdirectory(operator_modules)
add_subdirectory(test)
add_subdirectory(upns_interface)
if(WITH_VISUALIZATION)
add_subdirectory(visualization)
endif(WITH_VISUALIZATION)

# make data visible in qtcreator
add_custom_target(resourceFiles ALL ${CMAKE_COMMAND} -E echo "nop" SOURCES ${Datas_All})

#target_link_libraries(${PROJECT_NAME} )

# make qml visible in qtcreator
file(GLOB_RECURSE UPNS_QMLFILES ${PROJECT_SOURCE_DIR}/*.qml ${PROJECT_SOURCE_DIR}/*/qmldir)

add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add qml files to project" SOURCES ${ECS_QMLFILES})
