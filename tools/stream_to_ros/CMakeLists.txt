if(NOT DISABLE_PCL)
  project(stream_to_ros)
  cmake_minimum_required(VERSION 2.8)

  # message colors
  if(NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(Red         "${Esc}[31m")
    set(Green       "${Esc}[32m")
  endif()

  find_package(roscpp)
  find_package(rosgraph_msgs)
  find_package(sensor_msgs)

  if(NOT (roscpp_FOUND
          AND rosgraph_msgs_FOUND
          AND sensor_msgs_FOUND))
    message(AUTHOR_WARNING "${Red}"
      "Tool \"${PROJECT_NAME}\" will not be build, dependencies are not satisfied"
      "\n - roscpp: ${roscpp_FOUND}"
      "\n - rosgraph_msgs: ${rosgraph_msgs_FOUND}"
      "\n - sensor_msgs: ${sensor_msgs_FOUND}"
      "${ColourReset}")
  else()
    message(AUTHOR_WARNING "${Green}"
      "Tool \"${PROJECT_NAME}\" will be build"
      "${ColourReset}")

    # describe target
    # make proto visible in qtcreator
    file(GLOB_RECURSE YAML_FILES
      ${PROJECT_SOURCE_DIR}/*.yaml
      )
    file(GLOB_RECURSE UPNS_SOURCES
      ${PROJECT_SOURCE_DIR}/src/*.cpp
      ${PROJECT_SOURCE_DIR}/src/*.h
      )
    add_executable(${PROJECT_NAME} ${UPNS_SOURCES})
    set_target_properties(${PROJECT_NAME} PROPERTIES
      CXX_STANDARD 14
      )
    target_link_libraries(${PROJECT_NAME}
      stdc++
      )

    # mapit
    find_package(mapit REQUIRED)
    target_link_libraries(${PROJECT_NAME}
      ${mapit_interface}
      ${mapit_interface_cpp}
      ${mapit_standard_repository_factory}
      layertype_pointcloud2
      layertype_tf
      )

    # ROS
    #find_package(roscpp REQUIRED)
    #find_package(rosgraph_msgs REQUIRED)
    #find_package(sensor_msgs REQUIRED)
    target_include_directories(${PROJECT_NAME} PRIVATE
      ${roscpp_INCLUDE_DIRS}
      ${rosgraph_msgs_INCLUDE_DIRS}
      ${sensor_msgs_INCLUDE_DIRS}
      )
    target_link_libraries(${PROJECT_NAME}
      ${roscpp_LIBRARIES}
      ${rosgraph_msgs_LIBRARIES}
      ${sensor_msgs_LIBRARIES}
      )

    # boost
    find_package(Boost COMPONENTS program_options REQUIRED)
    target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES}
      )

    # log4cplus
    if(WITH_LOG4CPLUS)
      target_link_libraries(${PROJECT_NAME} ${LOG4CPLUS_LIBRARIES})
    endif(WITH_LOG4CPLUS)

    # build structure
    execute_process(
      COMMAND "${CMAKE_COMMAND}" "-E" "make_directory" ${CMAKE_BINARY_DIR}/tools/bin
      )
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

    install(TARGETS ${PROJECT_NAME} DESTINATION bin/upns_tools)
  endif()
endif()
