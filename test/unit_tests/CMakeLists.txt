project(upns_test)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

set(CMAKE_AUTOMOC ON)

find_package(Qt5Test ${UPNS_QTVERSION} REQUIRED)

INCLUDE_DIRECTORIES( UPNS_ROOT_DIR )

if(NOT (DISABLE_PCL))
find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
else( NOT (DISABLE_PCL))
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#set(PCL_CUSTOM_ROOT ${UPNS_ROOT_DIR}/externals/pcl-custom/include)
#include_directories(${PCL_CUSTOM_ROOT})
#FILE(GLOB PCL_CUSTOM_SRC ${PCL_CUSTOM_ROOT}/impl/* ${PCL_CUSTOM_ROOT}/common/impl/* ${PCL_CUSTOM_ROOT}/io/impl/*)
#set(PCL_CUSTOM_LIB ${Boost_SYSTEM_LIBRARY})
endif (NOT (DISABLE_PCL))

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
SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR}/src)
#list(REMOVE_ITEM SUBDIRS "common")
#FILE(GLOB_RECURSE ALL_TEST_SRC ${CMAKE_CURRENT_SOURCE_DIR}/unit_tests/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/unit_tests/*.h )

SET(AllUnitTest "")
SET(AllUnitTestCPP "")

FOREACH(subdir ${SUBDIRS})
  FILE(GLOB CurrentUnitTestH ${CMAKE_CURRENT_SOURCE_DIR}/src/${subdir}/*.h)
  FILE(GLOB CurrentUnitTestCPP ${CMAKE_CURRENT_SOURCE_DIR}/src/${subdir}/*.cpp)
  SET(AllUnitTestH ${AllUnitTestH} ${CurrentUnitTestH})
  SET(AllUnitTestCPP ${AllUnitTestCPP} ${CurrentUnitTestCPP})

  add_executable(${subdir} ${TEST_MAIN} ${CurrentUnitTestH} ${CurrentUnitTestCPP} ${PCL_CUSTOM_SRC})
  target_link_libraries(${subdir} Qt5::Test upns_mapmanager upns_interface layertype_pointcloud2 ${PCL_LIBRARIES} ${PCL_CUSTOM_LIB} )

add_custom_command(
    TARGET ${subdir} POST_BUILD
    COMMAND ln -sfT ${UPNS_ROOT_DIR}/data ${CMAKE_BINARY_DIR}/test/data
)

add_custom_command(
    TARGET ${subdir} POST_BUILD
    COMMAND ln -sfT ${UPNS_ROOT_DIR}/logging.properties ${CMAKE_BINARY_DIR}/test/logging.properties
)
ENDFOREACH()

add_executable(TestAll ${TEST_MAIN} ${AllUnitTestH} ${AllUnitTestCPP} ${PCL_CUSTOM_SRC} )
target_link_libraries(TestAll Qt5::Test upns_mapmanager upns_interface layertype_pointcloud2 ${PCL_LIBRARIES} ${PCL_CUSTOM_LIB} )
add_test(TestAll ${EXECUTABLE_OUTPUT_PATH}/TestAll)