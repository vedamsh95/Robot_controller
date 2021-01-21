add_library(LIN
            lin.cpp
        )

target_include_directories(LIN
        PUBLIC
        "${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
        "${CMAKE_SOURCE_DIR}/src/ctrl/kinematics/direct"
        "${CMAKE_SOURCE_DIR}/src/ctrl/kinematics/inverse"
        "${CMAKE_SOURCE_DIR}/src/ctrl"
        )