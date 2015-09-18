IF(CMAKE_BUILD_TYPE MATCHES Release)

# RELEASE flags

    ADD_DEFINITIONS(-DRELEASE)
    #ADD_DEFINITIONS(-DQT_NO_DEBUG_OUTPUT)
    ADD_DEFINITIONS(-DQT_NO_DEBUG)

    if(CMAKE_COMPILER_IS_GNUCXX)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wreturn-type -Werror")  ## Optimize -Wall -Wextra
        #set(CMAKE_CXX_FLAGS "-O1 -pedantic-errors -Wreturn-type -Werror -Wall -Wextra -Wno-unused -Wno-long-long")  ## Optimize
        #set(CMAKE_CXX_FLAGS "-O1 -Wreturn-type -fpermissive -Wl,--no-as-needed -Wl,--no-undefined")  ## Optimize
    endif()

    message(STATUS "Debugging disabled -> release build")

ELSE(CMAKE_BUILD_TYPE MATCHES Release)

# DEBUG flags

    set(CMAKE_BUILD_TYPE Debug)
    #ADD_DEFINITIONS(-DDEBUG)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -Wreturn-type -Werror") # -Wall -Wextra
    #set(CMAKE_CXX_FLAGS "-O0 -pedantic-errors -Wreturn-type -Werror -Wall -Wextra -Wno-unused -Wno-long-long")
    #set(CMAKE_CXX_FLAGS "-O0 -Wreturn-type -fpermissive -Wl,--no-as-needed -Wl,--no-undefined")

    message(STATUS "Debugging enabled -> debug build")

ENDIF(CMAKE_BUILD_TYPE MATCHES Release)
