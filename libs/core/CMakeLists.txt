project(core)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

find_package(Boost REQUIRED COMPONENTS filesystem iostreams)

file(GLOB_RECURSE UPNS_MM_SOURCES src/*.cpp
                                  src/*.h
                                  src/serialization/file_system/*.cpp
                                  src/serialization/file_system/*.h
                                  include/*.h
                                  src/sha/*.cpp
                                  src/sha/*.h)

add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${UPNS_MM_SOURCES} )
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

#Note: Windows (and also linux in some cases?):
# PCL was linked statically against boost. PCL is linked statically to layertype.
# Layertype has nothing to do with core. (linked dynamic and at runtime)
# Somehow boost dependency comes in here. I guess a static library wants it, thus the executable/dynamic-library must have it.
target_link_libraries(${PROJECT_NAME} interface interface_cpp ${Boost_LIBRARIES} ${BOOST_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY})

if(WITH_LOG4CPLUS)
  target_link_libraries(${PROJECT_NAME} ${LOG4CPLUS_LIBRARIES})
endif(WITH_LOG4CPLUS)

if(NOT WIN32)
    target_link_libraries(${PROJECT_NAME} dl)
endif(NOT WIN32)
#    E:/devel/boost_1_55_0/lib64-msvc-12.0/libboost_filesystem-vc120-mt-gd-1_55.lib
#    E:/devel/boost_1_55_0/lib64-msvc-12.0/libboost_system-vc120-mt-gd-1_55.lib
#    E:/devel/boost_1_55_0/lib64-msvc-12.0/libboost_date_time-vc120-mt-gd-1_55.lib)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
    $<INSTALL_INTERFACE:include>
  PRIVATE
    src
    ${Boost_INCLUDE_DIRS}
)

include(GenerateExportHeader)

generate_export_header(${PROJECT_NAME})
set_property(TARGET ${PROJECT_NAME} PROPERTY VERSION 1)
set_property(TARGET ${PROJECT_NAME} PROPERTY SOVERSION 1)
set_property(TARGET ${PROJECT_NAME} PROPERTY
  INTERFACE_mapit_core_MAJOR_VERSION 1)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING mapit_MAJOR_VERSION
)

install(TARGETS ${PROJECT_NAME} EXPORT upns_core_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(DIRECTORY include DESTINATION include )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_coreConfigVersion.cmake"
  VERSION 1
  COMPATIBILITY AnyNewerVersion
)

# TODO: CMAKE_BINARY_DIR or CMAKE_CURRENT_BINARY_DIR?
export(EXPORT upns_core_targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/mapit_coreTargets.cmake"
  NAMESPACE mapit::
)

configure_file(cmake/mapitCoreConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_coreConfig.cmake"
  @ONLY
)

set(ConfigPackageLocation lib/cmake/mapit)
install(EXPORT upns_core_targets
  FILE
    mapitCoreTargets.cmake
  NAMESPACE
    mapit::
  DESTINATION
    ${ConfigPackageLocation}
)
install(
  FILES
    cmake/mapitCoreConfig.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/mapit_coreConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)
