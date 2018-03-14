################################################################################
#
# Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
#           2015-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
#
################################################################################

#  This file is part of mapit.
#
#  Mapit is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  Mapit is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
#

project(upns_test)
cmake_minimum_required(VERSION 3.2)
aux_source_directory(. SRC_LIST)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Test REQUIRED)

include_directories( MAPIT_ROOT_DIR )

find_package(mapit REQUIRED)
#find_package(assimp REQUIRED)
if(MAPIT_ENABLE_OPENVDB)
  find_package(OpenVDB REQUIRED)
endif(MAPIT_ENABLE_OPENVDB)
if(MAPIT_ENABLE_PCL)
  find_package(TBB REQUIRED)
  find_package(PCL QUIET REQUIRED COMPONENTS common io filters)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
endif(MAPIT_ENABLE_PCL)

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
#TODO: include these again!
list(REMOVE_ITEM SUBDIRS "entitydata_test")
list(REMOVE_ITEM SUBDIRS "interface_test")
#list(REMOVE_ITEM SUBDIRS "operators_test")
#list(REMOVE_ITEM SUBDIRS "messages_test")
#FILE(GLOB_RECURSE ALL_TEST_SRC ${CMAKE_CURRENT_SOURCE_DIR}/unit_tests/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/unit_tests/*.h )

SET(AllUnitTest "")
SET(AllUnitTestCPP "")

FOREACH(subdir ${SUBDIRS})
  FILE(GLOB CurrentUnitTestH ${CMAKE_CURRENT_SOURCE_DIR}/src/${subdir}/*.h)
  FILE(GLOB CurrentUnitTestCPP ${CMAKE_CURRENT_SOURCE_DIR}/src/${subdir}/*.cpp)
  SET(AllUnitTestH ${AllUnitTestH} ${CurrentUnitTestH})
  SET(AllUnitTestCPP ${AllUnitTestCPP} ${CurrentUnitTestCPP})

  add_executable(${subdir} ${TEST_MAIN} ${CurrentUnitTestH} ${CurrentUnitTestCPP} )

  set_property(TARGET ${subdir} PROPERTY CXX_STANDARD 11)
  target_link_libraries(${subdir} ${Qt5Test_LIBRARIES}
                                  ${mapit_core}
                                  ${mapit_interface}
                                  ${mapit_interface_cpp}
                                  ${mapit_networking_node}
                                  ${mapit_standard_repository_factory}
                                  layertype_pointcloud2
                                  load_pointcloud
                                  layertype_tf
                                  ${PCL_LIBRARIES} ${BOOST_LIBRARIES} ${LOG4CPLUS_LIBRARIES}
                                  ${OPENVDB_LIBRARIES} ${TBB_LIBRARIES} ${HALF_LIBRARIES} stdc++)

  if(MAPIT_ENABLE_OPENVDB)
    target_link_libraries(${subdir} layertype_openvdb)
  endif(MAPIT_ENABLE_OPENVDB)
  #custom_target_use_boost(${subdir})
  find_package(Boost COMPONENTS filesystem system date_time)

  execute_process(
    COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${MAPIT_ROOT_DIR}/data ${CMAKE_BINARY_DIR}/test/unit_tests/data
  )
  execute_process(
    COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/test/unit_tests/libs
  )
  execute_process(
    COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${MAPIT_ROOT_DIR}/logging.properties ${CMAKE_BINARY_DIR}/test/unit_tests/logging.properties
  )
#add_custom_command(
#    TARGET ${subdir} POST_BUILD
#    COMMAND ln -sfT ${MAPIT_ROOT_DIR}/data ${CMAKE_BINARY_DIR}/test/unit_tests/data
#)

#add_custom_command(
#    TARGET ${subdir} POST_BUILD
#    COMMAND ln -sfT ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/test/unit_tests/libs
#)

#add_custom_command(
#    TARGET ${subdir} POST_BUILD
#    COMMAND ln -sfT ${MAPIT_ROOT_DIR}/logging.properties ${CMAKE_BINARY_DIR}/test/unit_tests/logging.properties
#)
ENDFOREACH()

add_executable(TestAll ${TEST_MAIN} ${AllUnitTestH} ${AllUnitTestCPP} )
#custom_target_use_boost(${subdir})
find_package(Boost  COMPONENTS filesystem system date_time REQUIRED)

target_link_libraries(TestAll Qt5::Test
                              ${mapit_core}
                              ${mapit_interface}
                              ${mapit_interface_cpp}
                              ${mapit_networking_node}
                              ${mapit_standard_repository_factory}
                              layertype_pointcloud2
                              load_pointcloud
                              layertype_tf
                              ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${LOG4CPLUS_LIBRARIES}
                              ${OPENVDB_LIBRARIES} ${TBB_LIBRARIES} ${HALF_LIBRARIES} stdc++)

if(MAPIT_ENABLE_OPENVDB)
target_link_libraries(TestAll levelset_to_mesh layertype_openvdb)
add_definitions(-DMAPIT_ENABLE_OPENVDB)
endif(MAPIT_ENABLE_OPENVDB)
#if(WIN32)
#target_link_libraries(TestAll
#    C:/boost_1_56_0/lib64-msvc-12.0/libboost_system-vc120-mt-1_56.lib
#    C:/boost_1_56_0/lib64-msvc-12.0/libboost_date_time-vc120-mt-1_56.lib
#    C:/boost_1_56_0/lib64-msvc-12.0/libboost_filesystem-vc120-mt-1_56.lib)
#endif()
add_test(TestAll ${EXECUTABLE_OUTPUT_PATH}/TestAll)
