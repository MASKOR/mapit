project(scripts)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

# This should create the "tools/bin" directory, that can be used to execute tools directly.
# Moreover it should copy example scripts to the folder.
# However this is not trivial using the beloved CMake.
# Workaround: Each tools creates and ensures this manually, so no loop over all tools is required.

execute_process(
    COMMAND "${CMAKE_COMMAND}" "-E" "make_directory" ${CMAKE_BINARY_DIR}/tools/bin
)

FILE(GLOB children RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)
list(REMOVE_ITEM children "scripts")
SET(dirlist "")
FOREACH(child ${children})
  IF(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${child})
    SET(dirlist ${dirlist} ${child})
  ELSE()
      execute_process(
          COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_CURRENT_SOURCE_DIR}/${child} ${CMAKE_BINARY_DIR}/tools/bin/${PROJECT_NAME}
      )
  ENDIF()
ENDFOREACH()

add_custom_target(ToolBinaryInstall)

add_dependencies(ToolBinaryInstall checkout2filesystem checkout_create commit execute_operator mapitd)
