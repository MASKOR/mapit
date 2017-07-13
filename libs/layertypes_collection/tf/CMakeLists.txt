project(layertype_tf)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${PROJECT_SOURCE_DIR}/../../mapmanager/include/)
if(WITH_LOG4CPLUS)
  include_directories(${PROJECT_SOURCE_DIR}/../../../externals/log4cplus/include/)
endif(WITH_LOG4CPLUS)
if(WIN32)
include_directories(${PROJECT_SOURCE_DIR}/../../../externals/windows/include/)
endif(WIN32)

include(FindLog4cplus)

find_package(Protobuf)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})

find_package(mapit)

# Eigen3  Will just not be found on windows
if(WIN32)
    find_package(Eigen3)
    set(EIGEN3_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/externals/eigen3) # contains Eigen/...
else(WIN32)
    find_package(Eigen3 REQUIRED)
endif(WIN32)

# TODO: If language could not be determined: There are Additional cmake variable for boost that mus be set. Fix this
file(GLOB_RECURSE LAYERTYPE_TF_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/include/**.h )
add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${LAYERTYPE_TF_SOURCES} )
target_link_libraries( ${PROJECT_NAME} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})

find_package(Boost COMPONENTS serialization system filesystem date_time)
target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${mapit_interface} ${mapit_interface_cpp})
target_link_libraries(${PROJECT_NAME} ${LOG4CPLUS_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS}
                                                  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                  $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME} PRIVATE PROJECT_NAME="${PROJECT_NAME}"
                                           PUBLIC  TfEntitydata_TYPENAME="${PROJECT_NAME}")

if(UNIX)
  # tell CMake Prefix is "lib" otherwise it can not be queried
  set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "lib")
endif(UNIX)
get_target_property(OLD_PREFIX ${PROJECT_NAME} PREFIX)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX ${OLD_PREFIX}${UPNS_INSTALL_LAYERTYPES})

#install(TARGETS ${PROJECT_NAME} DESTINATION lib)
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${CMAKE_LIBRARY_ARCHITECTURE} )

include(GenerateExportHeader)

generate_export_header(${PROJECT_NAME})
#set_property(TARGET ${PROJECT_NAME} PROPERTY VERSION ${MAPIT_VERSION})
#set_property(TARGET ${PROJECT_NAME} PROPERTY SOVERSION ${MAPIT_VERSION})
set_property(TARGET ${PROJECT_NAME} PROPERTY
  INTERFACE_mapit_layertype_tf_MAJOR_VERSION ${MAPIT_MAJOR_VERSION})
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING MAPIT_MAJOR_VERSION
)

install(TARGETS ${PROJECT_NAME} EXPORT mapit_layertype_tf_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(DIRECTORY include DESTINATION include )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_layertype_tfConfigVersion.cmake"
  VERSION 1
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT mapit_layertype_tf_targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/mapit_layertype_tfTargets.cmake"
  NAMESPACE mapit::
)
configure_file(cmake/mapit_layertype_tfConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_layertype_tfConfig.cmake"
  COPYONLY
)

set(ConfigPackageLocation lib/cmake/mapit)
install(EXPORT mapit_layertype_tf_targets
  FILE
    layertype_tfTargets.cmake
  NAMESPACE
    mapit::
  DESTINATION
    ${ConfigPackageLocation}
)
install(
  FILES
    cmake/mapit_layertype_tfConfig.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/layertype_tfConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)
