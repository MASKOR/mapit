# Locate json11
#
# This module defines
#  JSON11_FOUND, if false, do not try to link to json11
#  JSON11_LIBRARY, where to find json11
#  JSON11_INCLUDE_DIR, where to find json11.hpp
#
# If json11 is not installed in a standard path, you can use the JSON11_DIR CMake variable
# to tell CMake where json11 is.

find_path(JSON11_INCLUDE_DIR json11.hpp
          PATHS
          /usr/local/include/
          /usr/include/
          /sw/json11/         # Fink
          /opt/local/json11/  # DarwinPorts
          /opt/csw/json11/    # Blastwave
          /opt/json11/
          /include/)

if(NOT WIN32)
    SET(CMAKE_FIND_LIBRARY_SUFFIXES ".so" ".a")
endif(NOT WIN32)

find_library(JSON11_LIBRARY
             NAMES  json11
             PATH_SUFFIXES lib64 lib
             PATHS  /usr/local
                    /usr
                    /sw
                    /opt/local
                    /opt/csw
                    /opt
                    /lib)

# handle the QUIETLY and REQUIRED arguments and set JSON11_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(JSON11 DEFAULT_MSG JSON11_INCLUDE_DIR JSON11_LIBRARY)
mark_as_advanced(JSON11_INCLUDE_DIR JSON11_LIBRARY)
