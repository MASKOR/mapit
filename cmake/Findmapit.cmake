#TODO: does not yet support visualization

set(mapit_DIR CACHE PATH "Hint where mapit can be found")

find_package(PkgConfig)
pkg_check_modules(PC_MAPIT QUIET mapit)
set(MAPIT_DEFINITIONS ${PC_MAPIT_CFLAGS_OTHER})

find_path(MAPIT_INCLUDE_DIR upns/abstractentitydata.h
          HINTS ${mapit_DIR}/include ${PC_MAPIT_INCLUDEDIR} ${PC_MAPIT_INCLUDE_DIRS} )

find_path(MAPIT_LIBRARY_PATH upns_core
          HINTS ${mapit_DIR}/lib ${PC_MAPIT_LIBDIR} ${PC_MAPIT_LIBRARY_DIRS})

find_path(MAPIT_LAYERTYPE_PATH libupns_operators_voxelgridfilter.so
          HINTS ${mapit_DIR}/lib ${PC_MAPIT_LIBDIR} ${PC_MAPIT_LIBRARY_DIRS})

find_library(MAPIT_CORE_LIBRARY NAMES upns_core
             PATHS ${mapit_DIR}/lib
             HINTS ${PC_MAPIT_LIBDIR} ${PC_MAPIT_LIBRARY_DIRS})

find_library(MAPIT_NETWORKING_LIBRARY NAMES upns_networking_node
             PATHS ${mapit_DIR}/lib
             HINTS ${PC_MAPIT_LIBDIR} ${PC_MAPIT_LIBRARY_DIRS})

find_library(MAPIT_STD_REPO_FACTORY_LIBRARY NAMES upns_standard_repository_factory
             PATHS ${mapit_DIR}/lib
             HINTS ${PC_MAPIT_LIBDIR} ${PC_MAPIT_LIBRARY_DIRS})


find_library(MAPIT_INTERFACE_LIBRARY NAMES upns_interface
             PATHS ${mapit_DIR}/lib
             HINTS ${PC_MAPIT_LIBDIR} ${PC_MAPIT_LIBRARY_DIRS})


include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set MAPIT_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(mapit DEFAULT_MSG
                                  MAPIT_LIBRARY MAPIT_INCLUDE_DIR)

mark_as_advanced(MAPIT_INCLUDE_DIR
                 MAPIT_INCLUDE_DIRS
                 MAPIT_LIBRARY
                 MAPIT_LIBRARIES
                 MAPIT_LIBRARY_PATH
                 MAPIT_LAYERTYPE_PATH
                 MAPIT_CORE_LIBRARY
                 MAPIT_NETWORKING_LIBRARY
                 MAPIT_STD_REPO_FACTORY_LIBRARY
                 MAPIT_INTERFACE_LIBRARY
)

set(UPNS_INSTALL_LAYERTYPES "upns_layertypes_") # prefix for shared libraries
set(UPNS_INSTALL_OPERATORS "upns_operators_") # prefix for shared libraries

set(MAPIT_LIBRARY ${MAPIT_CORE_LIBRARY} ${MAPIT_NETWORKING_LIBRARY} ${MAPIT_STD_REPO_FACTORY_LIBRARY} ${MAPIT_INTERFACE_LIBRARY} )

set(MAPIT_LIBRARIES ${MAPIT_LIBRARY} )
set(MAPIT_INCLUDE_DIRS ${MAPIT_INCLUDE_DIR} )
