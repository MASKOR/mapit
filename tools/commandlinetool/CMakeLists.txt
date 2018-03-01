################################################################################
#
# Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
#           2015-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
#
################################################################################

#  This file is part of mapit.
#
#  Mapit is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  Mapit is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
#

project(commandlinetool)
cmake_minimum_required(VERSION 2.8)

  # message colors
  if(NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(Red         "${Esc}[31m")
    set(Green       "${Esc}[32m")
  endif()

#file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)
file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/script/*)
add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add bash script files to project" SOURCES ${UPNS_SOURCES})

#add_executable(${PROJECT_NAME} ${UPNS_SOURCES})

#target_link_libraries(${PROJECT_NAME} upns_core standard_repository_factory ${mapit_networking_node} ${ZeroMQ_LIBRARIES} ${Boost_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} stdc++)
#if(NOT WIN32)
#  target_link_libraries(${PROJECT_NAME} uuid ${libzip})
#endif(NOT WIN32)

execute_process(
    COMMAND "${CMAKE_COMMAND}" "-E" "make_directory" ${CMAKE_BINARY_DIR}/tools/bin
)

execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/tools/${PROJECT_NAME}/libs
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${PROJECT_SOURCE_DIR}/src/repo.yaml ${CMAKE_BINARY_DIR}/tools/${PROJECT_NAME}/repo.yaml
)

execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/libs ${CMAKE_BINARY_DIR}/tools/bin/libs
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${PROJECT_SOURCE_DIR}/src/repo.yaml ${CMAKE_BINARY_DIR}/tools/bin/repo.yaml
)
execute_process(
    COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_BINARY_DIR}/tools/${PROJECT_NAME}/${PROJECT_NAME} ${CMAKE_BINARY_DIR}/tools/bin/${PROJECT_NAME}
)

#install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(PROGRAMS script/mapit DESTINATION bin )

find_package(bash-completion)
if(NOT BASH_COMPLETION_FOUND)
  message(AUTHOR_WARNING "${Red}"
    "Can't find bash-completion"
    "${ColourReset}")
else()
  message(AUTHOR_WARNING "${Green}"
    "Found bash-completion:\n${BASH_COMPLETION_COMPLETIONSDIR}"
    "${ColourReset}")
  install(PROGRAMS completion/mapit DESTINATION ${BASH_COMPLETION_COMPLETIONSDIR} )
endif()
