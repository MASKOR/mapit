project(standard_repository_factory)
cmake_minimum_required(VERSION 2.8)

find_package(Boost COMPONENTS program_options)

#zeromq
find_package(ZMQ REQUIRED)
include_directories(${ZMQ_INCLUDE_DIR})

#cppzeromq
find_package(CPPZMQ REQUIRED)
include_directories(${CPPZMQ_INCLUDE_DIR})

find_package(mapit REQUIRED)

set(WITH_YAML ON CACHE BOOL "Enable Yaml for repository configuration")
if(WITH_YAML)
  #yamlcpp
  find_package(YamlCpp)
  include_directories(${YAMLCPP_INCLUDE_DIR})
endif(WITH_YAML)

file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h ${PROJECT_SOURCE_DIR}/include/*.h )

add_library(${PROJECT_NAME} SHARED ${UPNS_SOURCES})

target_include_directories(${PROJECT_NAME}
        PUBLIC
           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
        PRIVATE
           src
)

target_link_libraries(${PROJECT_NAME} ${mapit_core} ${mapit_interface} ${mapit_interface_cpp} ${mapit_networking_node} ${ZMQ_LIBRARIES} ${Boost_LIBRARIES} ${YAMLCPP_LIBRARY})

if(WITH_LOG4CPLUS)
  target_link_libraries(${PROJECT_NAME} ${LOG4CPLUS_LIBRARIES})
endif(WITH_LOG4CPLUS)

if(NOT WIN32)
    target_link_libraries(${PROJECT_NAME} dl)
endif(NOT WIN32)

if(NOT WIN32)
  target_link_libraries(${PROJECT_NAME} uuid ${libzip})
endif(NOT WIN32)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME upns_${PROJECT_NAME})

#install(TARGETS ${PROJECT_NAME} DESTINATION lib/${CMAKE_LIBRARY_ARCHITECTURE})
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${CMAKE_LIBRARY_ARCHITECTURE} )

include(GenerateExportHeader)

generate_export_header(${PROJECT_NAME})
#set_property(TARGET ${PROJECT_NAME} PROPERTY VERSION ${MAPIT_VERSION})
#set_property(TARGET ${PROJECT_NAME} PROPERTY SOVERSION ${MAPIT_VERSION})
set_property(TARGET ${PROJECT_NAME} PROPERTY
  INTERFACE_mapit_standard_repository_factory_MAJOR_VERSION ${MAPIT_MAJOR_VERSION})
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING MAPIT_MAJOR_VERSION
)

install(TARGETS ${PROJECT_NAME} EXPORT mapit_standard_repository_factory_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(DIRECTORY include DESTINATION include )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_standard_repository_factoryConfigVersion.cmake"
  VERSION 1
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT mapit_standard_repository_factory_targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/mapit_standard_repository_factoryTargets.cmake"
  NAMESPACE mapit::
)
configure_file(cmake/mapitStandardRepositoryFactoryConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_standard_repository_factoryConfig.cmake"
  COPYONLY
)

set(ConfigPackageLocation lib/cmake/mapit)
install(EXPORT mapit_standard_repository_factory_targets
  FILE
    standard_repository_factoryTargets.cmake
  NAMESPACE
    mapit::
  DESTINATION
    ${ConfigPackageLocation}
)
install(
  FILES
    cmake/mapitStandardRepositoryFactoryConfig.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/standard_repository_factoryConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)
