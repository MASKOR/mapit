project(upns_test)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

set(CMAKE_AUTOMOC ON)

find_package(Qt5Test ${UPNS_QTVERSION} REQUIRED)

INCLUDE_DIRECTORIES( UPNS_ROOT_DIR )

find_package(PCL 1.7 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#file(GLOB_RECURSE UPNS_TEST_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

#add_executable(${PROJECT_NAME} ${SRC_LIST} ${UPNS_TEST_SOURCES} )

#target_link_libraries(${PROJECT_NAME} Qt5::Core upns_mapmanager upns_interface )

#cmake_minimum_required(VERSION 2.8)

FILE(GLOB TEST_MAIN ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
        SET(dirlist ${dirlist} ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()
SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR}/unit_tests)

#list(REMOVE_ITEM SUBDIRS "common")

FOREACH(subdir ${SUBDIRS})
  FILE(GLOB CurrentUnitTestH ${CMAKE_CURRENT_SOURCE_DIR}/unit_tests/${subdir}/*.h)
  FILE(GLOB CurrentUnitTestCPP ${CMAKE_CURRENT_SOURCE_DIR}/unit_tests/${subdir}/*.cpp)

  add_executable(${subdir} ${TEST_MAIN} ${CurrentUnitTestH} ${CurrentUnitTestCPP})
  target_link_libraries(${subdir} Qt5::Test upns_mapmanager upns_interface layertype_pointcloud2 ${PCL_LIBRARIES} )
ENDFOREACH()
