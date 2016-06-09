# How is this framework loaded?
#  There is no find-script. Leveldb is strictly loaded from upns externals.
# How can be compiled manually?
#  On Windows do not follow the "windows" instructions but use cmake.
#  Linux:
#    make.
#    ignore debug building...

macro(custom_set_vars_leveldb)

    if (WIN32)
        set(LEVELDB_LIBRARY_DIRECTORY "${EXTERNALS_DIR}/windows/lib-msvc2013-md-64" CACHE FILEPATH "leveldb lib")
        set(LEVELDB_INCLUDE_DIRECTORY "${EXTERNALS_DIR}/windows/include" CACHE FILEPATH "leveldb lib")
        set(LEVELDB_LIB_RELEASE "${LEVELDB_LIBRARY_DIRECTORY}/libleveldb.lib" CACHE FILEPATH "leveldb lib")
        set(LEVELDB_LIB_DEBUG "${LEVELDB_LIBRARY_DIRECTORY}/libleveldbd.lib" CACHE FILEPATH "leveldb lib")
    else()
        set(LEVELDB_LIBRARY_DIRECTORY "${EXTERNALS_DIR}/leveldb/" CACHE FILEPATH "leveldb lib")
        set(LEVELDB_INCLUDE_DIRECTORY "${EXTERNALS_DIR}/leveldb/include" CACHE FILEPATH "leveldb lib")
        set(LEVELDB_LIB_RELEASE "${LEVELDB_LIBRARY_DIRECTORY}/libleveldb.so" CACHE FILEPATH "leveldb lib")
        set(LEVELDB_LIB_DEBUG "${LEVELDB_LIBRARY_DIRECTORY}/libleveldb.so" CACHE FILEPATH "leveldb lib") # Note: "d" is not there after default make. so don't use it by default
    endif()

    #link_directories(${EXTERNALS_DIR}/windows/lib-msvc2013-md-64)

    string( TOLOWER "${CMAKE_BUILD_TYPE}" LOWER_CMAKE_BUILD_TYPE )
    if (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
        set(LEVELDB_LIBRARIES ${LEVELDB_LIB_DEBUG})
    else()
        set(LEVELDB_LIBRARIES ${LEVELDB_LIB_RELEASE})
    endif()

endmacro(custom_set_vars_leveldb)

#leads to Cannot specify link l         ibraries for target "databasedump" which is not built by this project.
#macro(custom_target_use_leveldb TARGET)
#    custom_set_vars_leveldb()
#    include_directories(${LEVELDB_INCLUDE_DIRECTORY})
#    # if the target is has a public interface which requires all dependent targets to use leveldb (header):
#    #target_include_directory(#{LEVELDB_INCLUDE_DIRECTORY})
#    target_link_libraries(${TARGET} ${LEVELDB_LIBRARIES})
#endmacro(custom_target_use_leveldb)
