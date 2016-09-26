cmake_minimum_required(VERSION 2.8)


MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
        SET(dirlist ${dirlist} ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})

#list(REMOVE_ITEM SUBDIRS "common")

FOREACH(subdir ${SUBDIRS})
    ADD_SUBDIRECTORY(${subdir})
ENDFOREACH()
