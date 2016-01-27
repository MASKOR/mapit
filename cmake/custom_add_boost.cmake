# How is this framework loaded?
#  For linux this is easily the version installed by packetmanager.
#  It should not be needed to mix diferent versions of boost for any OS.
#
# How can be compiled manually?
#  On Windows install pre-built binaries for msvc12 (msvc12 is equal to msvc2013).
#  Manually building was messy on windows with ./b2. I tried to output x64, dynamic linked,
#  multithread libs and instead got something x86/32-bit libs out. Upns links dynamically againt
#  cpp/ msvc runtime libraries which is compiler option /MD or /MDd. The compiler option for static
#  linking the runtime is named "/MT". It is importend not to mess this with boost library naming,
#  where "mt" stands for Multithreaded!
#  Boost 1.55: For MSVC2013 boost-serialization is missing/incomaptible. Do not use.
#  Boost 1.59 and 1.60: Incompatible to PCL
#  Boost 1.56: Everything works fine!
# Naming:
#  Use the libs without "-s" (these are the dynamically linked to msvc-runtime ones). This requires users of upns to
#  install msvc runtime libs. Upns does this, because there are some prebuilt libraries which are only available
#  as /MD /MDd (TODO: which ones? zlib?). For Upns use the libs which do not start with "libboost" but with
#  "boost" (dynamic linking). This requires you to put the needed boost*.dll-files next to the binary or in a system
#  path. (static linking seems to work also, not yet tested enough. Position independend code must then be enabled)
#  "-gd" are debug libs for boost.

macro(custom_set_vars_boost)
    if (WIN32)
        set(BOOST_CUSTOM_PATH "E:\devel\boost_1_56_0\lib64-msvc-12.0" CACHE PATH "let it point to all the libs")
        if(BOOST_CUSTOM_PATH)
            set(Boost_CHRONO_LIBRARY_DEBUG "${BOOST_CUSTOM_PATH}/libboost_chrono-vc120-mt-gd-1_56.lib")
            set(Boost_CHRONO_LIBRARY_RELEASE "${BOOST_CUSTOM_PATH}/libboost_chrono-vc120-mt-1_56.lib")
            set(Boost_DATE_TIME_LIBRARY_DEBUG "${BOOST_CUSTOM_PATH}/libboost_date_time-vc120-mt-gd-1_56.lib")
            set(Boost_DATE_TIME_LIBRARY_RELEASE "${BOOST_CUSTOM_PATH}/libboost_date_time-vc120-mt-1_56.lib")
            set(Boost_FILESYSTEM_LIBRARY_DEBUG "${BOOST_CUSTOM_PATH}/libboost_filesystem-vc120-mt-gd-1_56.lib")
            set(Boost_FILESYSTEM_LIBRARY_RELEASE "${BOOST_CUSTOM_PATH}/libboost_filesystem-vc120-mt-1_56.lib")
            set(Boost_IOSTREAMS_LIBRARY_DEBUG "${BOOST_CUSTOM_PATH}/libboost_iostreams-vc120-mt-gd-1_56.lib")
            set(Boost_IOSTREAMS_LIBRARY_RELEASE "${BOOST_CUSTOM_PATH}/libboost_iostreams-vc120-mt-1_56.lib")
            set(Boost_SERIALIZATION_LIBRARY_DEBUG "${BOOST_CUSTOM_PATH}/libboost_serialization-vc120-mt-gd-1_56.lib")
            set(Boost_SERIALIZATION_LIBRARY_RELEASE "${BOOST_CUSTOM_PATH}/libboost_serialization-vc120-mt-1_56.lib")
            set(Boost_SYSTEM_LIBRARY_DEBUG "${BOOST_CUSTOM_PATH}/libboost_system-vc120-mt-gd-1_56.lib")
            set(Boost_SYSTEM_LIBRARY_RELEASE "${BOOST_CUSTOM_PATH}/libboost_system-vc120-mt-1_56.lib")
            set(Boost_THREAD_LIBRARY_DEBUG "${BOOST_CUSTOM_PATH}/libboost_thread-vc120-mt-gd-1_56.lib")
            set(Boost_THREAD_LIBRARY_RELEASE "${BOOST_CUSTOM_PATH}/libboost_thread-vc120-mt-1_56.lib")
        endif(BOOST_CUSTOM_PATH)

    else()

    endif()

    set(CUSTOM_LIBRARIES_LEVELDB leveldb)
endmacro(custom_set_vars_boost)

macro(custom_target_use_boost TARGET)
    custom_add_vars_leveldb()
    target_link_library(${TARGET} ${CUSTOM_LIBRARIES_LEVELDB})
endmacro(custom_target_use_boost)
