################################################################################
#
# Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
#                2016 Tobias Neumann	<t.neumann@fh-aachen.de>
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

project(checkout2filesystem)
cmake_minimum_required(VERSION 2.8)

find_package(mapit)

# make proto visible in qtcreator
file(GLOB_RECURSE YAML_FILES ${PROJECT_SOURCE_DIR}/*.yaml)
add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add yaml files to project" SOURCES ${YAML_FILES})


file(GLOB_RECURSE UPNS_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

add_executable(${PROJECT_NAME} ${UPNS_SOURCES})

if(WIN32)
  #target_link_libraries(${PROJECT_NAME} )
else(WIN32)
  target_link_libraries(${PROJECT_NAME} uuid ${libzip})
endif(WIN32)

find_package(Boost COMPONENTS iostreams REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

target_link_libraries(${PROJECT_NAME} ${mapit_interface} ${mapit_interface_cpp} ${mapit_standard_repository_factory} ${Boost_LIBRARIES} stdc++)

if(WITH_LOG4CPLUS)
  target_link_libraries(${PROJECT_NAME} ${LOG4CPLUS_LIBRARIES})
endif(WITH_LOG4CPLUS)

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

install(TARGETS ${PROJECT_NAME} DESTINATION bin/upns_tools)
