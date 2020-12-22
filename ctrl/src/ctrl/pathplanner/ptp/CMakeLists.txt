add_library(PTP
            ptp.cpp
        Single_trajectory.cpp Single_trajectory.h Max_vel_trajectory.cpp Max_vel_trajectory.h Trapezoidal_trajectory.cpp Trapezoidal_trajectory.h)

target_include_directories(FWKin
        PUBLIC
        "${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
        )