if(NOT DISABLE_PCL)
  cmake_minimum_required(VERSION 2.8)
  project(load_bags)

  # message colors
  if(NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(Red         "${Esc}[31m")
    set(Green       "${Esc}[32m")
  endif()

  # required, but disables this project without an error
  find_package(rosbag)
  find_package(sensor_msgs)

  if(NOT (rosbag_FOUND AND sensor_msgs_FOUND))
    message(AUTHOR_WARNING "${Red}"
      "Operator \"${PROJECT_NAME}\" will not be build, dependencies are not satisfied"
      "\n - rosbag: ${rosbag_FOUND}"
      "\n - sensor_msgs: ${sensor_msgs_FOUND}"
      "${ColourReset}")
  else()
    message(AUTHOR_WARNING "${Green}"
      "Operator \"${PROJECT_NAME}\" will be build"
      "${ColourReset}")

    set(OPERATOR_VERSION 1)
    add_definitions(-DOPERATOR_NAME="${PROJECT_NAME}")
    add_definitions(-DOPERATOR_VERSION=${OPERATOR_VERSION})

    # describe target
    file(GLOB_RECURSE UPNS_OPERATOR_SOURCES
      ${PROJECT_SOURCE_DIR}/src/*.cpp
      ${PROJECT_SOURCE_DIR}/src/*.h
      )
    add_library(${PROJECT_NAME} SHARED
      ${SRC_LIST}
      ${UPNS_OPERATOR_SOURCES}
      )
    set_target_properties(${PROJECT_NAME} PROPERTIES
      VERSION ${OPERATOR_VERSION}
      SOVERSION ${OPERATOR_VERSION}
      )
    target_include_directories(${PROJECT_NAME}
      PRIVATE
        ${PROJECT_SOURCE_DIR}/src/
      PUBLIC
        ${PROJECT_SOURCE_DIR}/../../mapmanager/include/
      )

    # mapit
    find_package(mapit REQUIRED)
    target_link_libraries(${PROJECT_NAME}
      ${mapit_core}
      ${mapit_interface}
      layertype_pointcloud2
      layertype_tf
      )

    # Qt
    set(CMAKE_AUTOMOC ON)
    find_package(Qt5Core REQUIRED)
    qt5_use_modules(${PROJECT_NAME} Core)

    # ROS
    #find_package(rosbag REQUIRED)
    #find_package(sensor_msgs REQUIRED)
    target_include_directories(${PROJECT_NAME} PRIVATE
      ${rosbag_INCLUDE_DIRS}
      ${sensor_msgs_INCLUDE_DIRS}
      )
    target_link_libraries(${PROJECT_NAME}
      ${rosbag_LIBRARIES}
      ${sensor_msgs_LIBRARIES}
      )

    # naming and installing
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

  endif()
endif(NOT DISABLE_PCL)
