project(upns_test)
cmake_minimum_required(VERSION 3.2)
aux_source_directory(. SRC_LIST)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Test ${UPNS_QTVERSION} REQUIRED)

include_directories( UPNS_ROOT_DIR )

find_package(mapit REQUIRED)
#find_package(assimp REQUIRED)
if(WITH_OPENVDB)
  find_package(OpenVDB REQUIRED)
endif(WITH_OPENVDB)
if(WITH_PCL)
  find_package(TBB REQUIRED)
  find_package(PCL QUIET REQUIRED COMPONENTS common io filters)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
endif(WITH_PCL)

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
list(REMOVE_ITEM SUBDIRS "mapfileservice_test")
list(REMOVE_ITEM SUBDIRS "mapmanager_test")
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

  if(WITH_OPENVDB)
    target_link_libraries(${subdir} layertype_openvdb)
  endif(WITH_OPENVDB)
  #custom_target_use_boost(${subdir})
  find_package(Boost COMPONENTS filesystem system date_time)

  execute_process(
    COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/data ${CMAKE_BINARY_DIR}/test/unit_tests/data
  )
  execute_process(
    COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/test/unit_tests/libs
  )
  execute_process(
    COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${UPNS_ROOT_DIR}/logging.properties ${CMAKE_BINARY_DIR}/test/unit_tests/logging.properties
  )
#add_custom_command(
#    TARGET ${subdir} POST_BUILD
#    COMMAND ln -sfT ${UPNS_ROOT_DIR}/data ${CMAKE_BINARY_DIR}/test/unit_tests/data
#)

#add_custom_command(
#    TARGET ${subdir} POST_BUILD
#    COMMAND ln -sfT ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/test/unit_tests/libs
#)

#add_custom_command(
#    TARGET ${subdir} POST_BUILD
#    COMMAND ln -sfT ${UPNS_ROOT_DIR}/logging.properties ${CMAKE_BINARY_DIR}/test/unit_tests/logging.properties
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

if(WITH_OPENVDB)
target_link_libraries(TestAll levelset_to_mesh layertype_openvdb)
add_definitions(-DWITH_OPENVDB)
endif(WITH_OPENVDB)
#if(WIN32)
#target_link_libraries(TestAll
#    C:/boost_1_56_0/lib64-msvc-12.0/libboost_system-vc120-mt-1_56.lib
#    C:/boost_1_56_0/lib64-msvc-12.0/libboost_date_time-vc120-mt-1_56.lib
#    C:/boost_1_56_0/lib64-msvc-12.0/libboost_filesystem-vc120-mt-1_56.lib)
#endif()
add_test(TestAll ${EXECUTABLE_OUTPUT_PATH}/TestAll)
