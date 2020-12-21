add_library(IVKin
            inverse_kinematics.cpp
	    IVKinPos.cpp
        )

target_include_directories(FWKin
        PUBLIC
        "${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
        )