# How is this framework loaded?
#  Findscript. Ubuntu repo can be used
# How can be compiled manually?
#  Windows:
#    log4cplus can be used from externals folder. Just open msvc10 solution in msvc12 (2013), convert, set x64 and compile!
#  Linux:
#    cmake cache vars: "optimized;" must be removed to overcome compiler errors.

macro(custom_set_vars_log4cplus)

#    if (WIN32)
#        set(LEVELDB_LIBRARY_DIRECTORY "${EXTERNALS_DIR}/windows/lib-msvc2013-md-64" CACHE FILEPATH "leveldb lib")
#        set(LEVELDB_INCLUDE_DIRECTORY "${EXTERNALS_DIR}/windows/include" CACHE FILEPATH "leveldb lib")
#        set(LEVELDB_LIB_RELEASE "${LEVELDB_LIBRARY_DIRECTORY}/libleveldb.lib" CACHE FILEPATH "leveldb lib")
#        set(LEVELDB_LIB_DEBUG "${LEVELDB_LIBRARY_DIRECTORY}/libleveldbd.lib" CACHE FILEPATH "leveldb lib")
#    else()
#        set(LEVELDB_LIBRARY_DIRECTORY "${EXTERNALS_DIR}/leveldb/" CACHE FILEPATH "leveldb lib")
#        set(LEVELDB_INCLUDE_DIRECTORY "${EXTERNALS_DIR}/leveldb/include" CACHE FILEPATH "leveldb lib")
#        set(LEVELDB_LIB_RELEASE "${LEVELDB_LIBRARY_DIRECTORY}/libleveldb.so" CACHE FILEPATH "leveldb lib")
#        set(LEVELDB_LIB_DEBUG "${LEVELDB_LIBRARY_DIRECTORY}/libleveldb.so" CACHE FILEPATH "leveldb lib") # Note: "d" is not there after default make. so don't use it by default
#    endif()

    #link_directories(${EXTERNALS_DIR}/windows/lib-msvc2013-md-64)

#    string( TOLOWER "${CMAKE_BUILD_TYPE}" LOWER_CMAKE_BUILD_TYPE )
#    if (LOWER_CMAKE_BUILD_TYPE STREQUAL "debug")
#        set(LEVELDB_LIBRARIES ${LEVELDB_LIB_DEBUG})
#    else()
#        set(LEVELDB_LIBRARIES ${LEVELDB_LIB_RELEASE})
#    endif()

endmacro(custom_set_vars_log4cplus)

macro(custom_target_use_log4cplus TARGET)
    custom_set_vars_log4cplus()
    include_directories(${LOG4CPLUS_LIBRARIES})
    target_link_libraries(${TARGET} ${LEVELDB_LIBRARIES})
endmacro(custom_target_use_log4cplus)
