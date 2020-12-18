cmake_minimum_required(VERSION 3.14)
project(sdir_ctrl2020)

set(CMAKE_CXX_STANDARD 11)

include_directories(postype ctrl/src/ctrl/postypes)
include_directories(json ctrl/include/json)
include_directories(vrep ctrl/include/vrep)
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
add_definitions(-DDO_NOT_USE_SHARED_MEMORY)
if (UNIX)
    SET(CMAKE_CXX_FLAGS -pthread)
    #run apt-get install libpthread-stubs0-dev
endif (UNIX)

add_subdirectory(ctrl/src/ctrl)

add_executable(sdir_ctrl2020 ctrl/src/main.cpp)

add_dependencies(sdir_ctrl2020 Ctrl)
target_link_libraries(sdir_ctrl2020 Ctrl)