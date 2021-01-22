add_library(IVKin
            inverse_kinematics.cpp
        )

target_include_directories(IVKin
        PUBLIC
        "${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
        "${CMAKE_SOURCE_DIR}/src/ctrl/Config"
        )