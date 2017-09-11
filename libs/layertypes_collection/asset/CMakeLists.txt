project(layertype_asset)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

find_package(Protobuf)
include_directories(${PROTOBUF_INCLUDE_DIRS})

#find_package(assimp REQUIRED)

find_package(mapit)

file(GLOB_RECURSE TINYPLY_SOURCES ${PROJECT_SOURCE_DIR}/tinyply/source/tinyply.cpp ${PROJECT_SOURCE_DIR}/tinyply/source/tinyply.h)

file(GLOB_RECURSE LAYERTYPE_ASSET_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h ${PROJECT_SOURCE_DIR}/include/*.h)

add_library(${PROJECT_NAME} SHARED ${SRC_LIST} ${TINYPLY_SOURCES} ${LAYERTYPE_ASSET_SOURCES})

target_link_libraries(${PROJECT_NAME} ${mapit_interface} ${mapit_interface_cpp})# ${assimp_LIBRARIES})


target_include_directories(${PROJECT_NAME} PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                   $<INSTALL_INTERFACE:include>
                                                   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tinyply/source>
                                                   $<INSTALL_INTERFACE:tinyply/source>
                                           PRIVATE ${PROJECT_SOURCE_DIR}/src/)

target_compile_definitions(${PROJECT_NAME} PRIVATE PROJECT_NAME="${PROJECT_NAME}"
                                           PUBLIC  AssetEntitydata_TYPENAME="${PROJECT_NAME}")

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
  INTERFACE_mapit_layertype_asset_MAJOR_VERSION ${MAPIT_MAJOR_VERSION})
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING MAPIT_MAJOR_VERSION
)

install(TARGETS ${PROJECT_NAME} EXPORT mapit_layertype_asset_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(DIRECTORY include DESTINATION include )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_layertype_assetConfigVersion.cmake"
  VERSION 1
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT mapit_layertype_asset_targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/mapit_layertype_assetTargets.cmake"
  NAMESPACE mapit::
)
configure_file(cmake/mapit_layertype_assetConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_layertype_assetConfig.cmake"
  COPYONLY
)

set(ConfigPackageLocation lib/cmake/mapit)
install(EXPORT mapit_layertype_asset_targets
  FILE
    mapit_layertype_assetTargets.cmake
  NAMESPACE
    mapit::
  DESTINATION
    ${ConfigPackageLocation}
)
install(
  FILES
    cmake/mapit_layertype_assetConfig.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/mapit_layertype_assetConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)
