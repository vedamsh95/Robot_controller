add_library(IVKin
            inverse_kinematics.cpp
	    IVKinPos.cpp
	    IVMovement.cpp
        )

target_include_directories(FWKin
        PUBLIC
        "${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
        )