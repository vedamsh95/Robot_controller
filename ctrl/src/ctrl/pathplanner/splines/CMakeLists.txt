add_library(SPLINES
            splines.cpp
        )

target_include_directories(IVKin
        PUBLIC
        "${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
	"${CMAKE_SOURCE_DIR}/src/ctrl/kinematics/inverse"

        )