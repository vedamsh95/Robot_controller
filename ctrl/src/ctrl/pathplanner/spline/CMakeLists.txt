add_library(Spline
            spline.cpp
        )

target_include_directories(LIN
        PUBLIC
        "${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
        )