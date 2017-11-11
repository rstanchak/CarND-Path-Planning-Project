project(Path_Planning)

cmake_minimum_required (VERSION 2.8)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)

if(${MYBUILD})
    # custom configuration for my Ubuntu 14.04 system
    add_definitions(-DCONFIG_UWS13_COMPAT)
    add_definitions(-DCONFIG_MYBUILD)
    target_link_libraries(path_planning ssl)

    # unit test configuration
	include_directories(src)

	#	add_subdirectory(ext/googletest)
	#add_subdirectory(test)
endif()
