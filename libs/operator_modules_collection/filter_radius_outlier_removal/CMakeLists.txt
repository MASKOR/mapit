if(NOT DISABLE_PCL)
    cmake_minimum_required(VERSION 3.0)

    # describe operator
    set(OPERATOR_NAME filter_radius_outlier_removal)
    set(OPERATOR_VERSION 1.0)
    project(${OPERATOR_NAME})
    include_directories(${PROJECT_SOURCE_DIR}/src/)
    add_definitions(-DOPERATOR_NAME="${OPERATOR_NAME}")
    add_definitions(-DOPERATOR_VERSION=${OPERATOR_VERSION})

    # pcl
    find_package(PCL QUIET 1.8 REQUIRED
        COMPONENTS filters)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

    # mapit
    find_package(mapit REQUIRED)

    # Qt
    find_package(Qt5Core REQUIRED)

    # describe target
    file(GLOB_RECURSE SOURCES
        ${PROJECT_SOURCE_DIR}/src/*.cpp)
    file(GLOB_RECURSE HEADERS
        ${PROJECT_SOURCE_DIR}/src/*.hpp)
    add_library(${PROJECT_NAME} SHARED
        ${SOURCES}
        ${HEADERS})
    set_target_properties(${PROJECT_NAME} PROPERTIES
        VERSION ${OPERATOR_VERSION}
        SOVERSION ${OPERATOR_VERSION}
        PREFIX ${CMAKE_SHARED_LIBRARY_PREFIX}${UPNS_INSTALL_OPERATORS})

    # linker
    target_link_libraries(${PROJECT_NAME}
        ${PCL_FILTERS_LIBRARIES}
        ${mapit_interface}
        ${mapit_core}
        layertype_pointcloud2)

    qt5_use_modules(${PROJECT_NAME} Core)

    install(TARGETS ${PROJECT_NAME}
        DESTINATION lib)
endif(NOT DISABLE_PCL)
