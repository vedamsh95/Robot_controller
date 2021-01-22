add_library(PosTypes
            Configuration.cpp
            SixDPos.cpp
            TMatrix.cpp
            Trajectory.cpp)

target_include_directories(PosTypes
        PUBLIC
        "${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
        "${CMAKE_SOURCE_DIR}/src/ctrl/Config"
        )