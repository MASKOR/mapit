# How is this framework loaded?
#  There is no find-script. Leveldb is strictly loaded from upns externals.
# How can be compiled manually?
#  On Windows do not follow the "windows" instructions but use cmake.

macro(custom_set_vars_leveldb)
    if (WIN32)
    link_directories(${EXTERNALS_DIR}/windows/lib-msvc2013-md-64)
    include_directories(${EXTERNALS_DIR}/windows/include)
    else()
    link_directories(${EXTERNALS_DIR}/leveldb)
    include_directories(${EXTERNALS_DIR}/leveldb/include)
    endif()

    set(CUSTOM_LIBRARIES_LEVELDB leveldb)
endmacro(custom_set_vars_leveldb)

macro(custom_target_use_leveldb TARGET)
    custom_add_vars_leveldb()
    target_link_library(${TARGET} ${CUSTOM_LIBRARIES_LEVELDB})
endmacro(custom_target_use_leveldb)
