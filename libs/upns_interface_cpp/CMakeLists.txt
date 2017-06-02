project(upns_interface_cpp)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

# upns_interface_cpp contains headers with virtual interfaces, which can be implemented by different libraries.

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
#include_directories(${PROJECT_SOURCE_DIR}/src/)
#include_directories(${PROJECT_SOURCE_DIR}/include/)
#include_directories(${UPNS_INTERFACE_INCLUDE})

file(GLOB_RECURSE UPNS_INTERFACE_CPP_SOURCES include/*.h)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include)

#shows files in qtcreator
add_custom_target(${PROJECT_NAME}_NOP ALL ${CMAKE_COMMAND} -E echo "show files in qtcreator" SOURCES ${UPNS_INTERFACE_CPP_SOURCES})

#install(TARGETS ${PROJECT_NAME} DESTINATION lib/${CMAKE_LIBRARY_ARCHITECTURE})
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include/${CMAKE_LIBRARY_ARCHITECTURE} )

include(GenerateExportHeader)

generate_export_header(${PROJECT_NAME})
#set_property(TARGET ${PROJECT_NAME} PROPERTY VERSION ${MAPIT_VERSION})
#set_property(TARGET ${PROJECT_NAME} PROPERTY SOVERSION ${MAPIT_VERSION})
set_property(TARGET ${PROJECT_NAME} PROPERTY
  INTERFACE_mapit_MAJOR_VERSION ${MAPIT_MAJOR_VERSION})
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING MAPIT_MAJOR_VERSION
)

install(TARGETS ${PROJECT_NAME} EXPORT upns_interface_cpp_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(DIRECTORY include DESTINATION include )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mapit/mapitInterfaceCppConfigVersion.cmake"
  VERSION 1
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT upns_interface_cpp_targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/mapit/mapitInterfaceCppTargets.cmake"
  NAMESPACE mapit::
)
configure_file(cmake/mapitInterfaceCppConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/mapit/mapitInterfaceCppConfig.cmake"
  COPYONLY
)

set(ConfigPackageLocation lib/cmake/mapit)
install(EXPORT upns_interface_cpp_targets
  FILE
    mapitInterfaceCppTargets.cmake
  NAMESPACE
    mapit::
  DESTINATION
    ${ConfigPackageLocation}
)
install(
  FILES
    cmake/mapitInterfaceCppConfig.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/mapit/mapitInterfaceCppConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)
