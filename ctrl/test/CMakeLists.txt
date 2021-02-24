project(tests)
cmake_minimum_required (VERSION 3.14.2)
cmake_policy(SET CMP0048 NEW)

# Do not use, for example, `-std=gnu++11`.
#add_compile_options(/std:c++latest)
add_definitions(-D_SILENCE_CXX17_CODECVT_HEADER_DEPRECATION_WARNING)
add_definitions(-D_HAS_AUTO_PTR_ETC=1)
add_definitions(-D_HAS_TR1_NAMESPACE=1)
add_definitions(-D_SILENCE_TR1_NAMESPACE_DEPRECATION_WARNING)
add_definitions(-D_SILENCE_CXX17_ITERATOR_BASE_CLASS_DEPRECATION_WARNING)

set(LINK_FLAGS ${LINK_FLAGS} "-Wl -whole-archive")

include_directories(
        ${gtest_SOURCE_DIR}/include
        ${gtest_SOURCE_DIR}
)

add_executable(main IKTest.cpp LinTest.cpp PTPTest.cpp SplineTest.cpp)

add_dependencies(main Ctrl)

target_include_directories(main
        PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/src/ctrl"
        "${CMAKE_SOURCE_DIR}/src/ctrl/kinematics/direct"
        "${CMAKE_SOURCE_DIR}/src/ctrl/kinematics/inverse"
        "${CMAKE_SOURCE_DIR}/src/ctrl/pathplanner/lin"
        "${CMAKE_SOURCE_DIR}/src/ctrl/pathplanner/ptp"
        )

target_link_directories(main PUBLIC "${CMAKE_BINARY_DIR}/src/opti")
target_link_libraries(main Ctrl LIN gtest gtest_main)

#add_subdirectory(../src/ctrl)


########################################################################################################################
#                                                                                                                      #
#                                                Test Setup                                                            #
#                                                                                                                      #
########################################################################################################################
# cmake code taken from
# https://github.com/google/googletest/tree/master/googletest#incorporating-into-an-existing-cmake-project
# Download and unpack googletest at configure time
if(MSVC)
    configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
            RESULT_VARIABLE result
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
    if(result)
        message(FATAL_ERROR "CMake step for googletest failed: ${result}")
    endif()
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
            RESULT_VARIABLE result
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
    if(result)
        message(FATAL_ERROR "Build step for googletest failed: ${result}")
    endif()

    # Prevent overriding the parent project's compiler/linker
    # settings on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    # Add googletest directly to our build. This defines
    # the gtest and gtest_main targets.
    add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
            ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
            EXCLUDE_FROM_ALL)

    # The gtest/gtest_main targets carry header search path
    # dependencies automatically when using CMake 2.8.11 or
    # later. Otherwise we have to add them here ourselves.
    if (CMAKE_VERSION VERSION_LESS 2.8.11)
        include_directories("${gtest_SOURCE_DIR}/include")
    endif()
endif()