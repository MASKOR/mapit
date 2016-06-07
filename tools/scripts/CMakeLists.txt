project(scripts)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

FILE(GLOB children RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)
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
