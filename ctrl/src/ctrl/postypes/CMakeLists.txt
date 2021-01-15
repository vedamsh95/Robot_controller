add_library(PosTypes
            Configuration.cpp
            SixDPos.cpp
            TMatrix.cpp
            Trajectory.cpp
        Spline.cpp Spline.h)

target_include_directories(FWKin
        PUBLIC
        "${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
        )