project(commandlinetool)
cmake_minimum_required(VERSION 2.8)

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
