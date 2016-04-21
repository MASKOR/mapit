# How is this framework loaded?
#  Findscript. Ubuntu repo can be used
# How can be compiled manually?
#  Windows:
#  Linux:

macro(custom_set_vars_yamlcpp)

    string( TOLOWER "${CMAKE_BUILD_TYPE}" LOWER_CMAKE_BUILD_TYPE )
    if (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
        set(YAML_CPP_LIBRARIES ${YAML_LIB_DEBUG})
    else (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
        set(YAML_CPP_LIBRARIES ${YAML_LIB_RELEASE})
    endif (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")

link_directories(${YAML_CPP_LIB_DIR})
include_directories(${YAML_CPP_INCLUDE})
endmacro(custom_set_vars_yamlcpp)

macro(custom_target_use_yamlcpp TARGET)
    custom_set_vars_yamlcpp()

endmacro(custom_target_use_yamlcpp)
