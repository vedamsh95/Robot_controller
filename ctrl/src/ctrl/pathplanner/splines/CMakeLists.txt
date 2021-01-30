add_library(SPLINES
            splines.cpp
        )


if(USE_PYTHON_PLOT)
    target_include_directories(SPLINES
            PUBLIC
            "${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
	    "${CMAKE_SOURCE_DIR}/src/ctrl/kinematics/inverse"
	    "${CMAKE_SOURCE_DIR}/src/ctrl/kinematics/inverse"
            PRIVATE
            ${Python3_INCLUDE_DIRS}
            ${Python3_NumPy_INCLUDE_DIRS}
            )
else()
    target_include_directories(SPLINES
            PUBLIC
			"${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
	    "${CMAKE_SOURCE_DIR}/src/ctrl/kinematics/inverse"
	    "${CMAKE_SOURCE_DIR}/src/ctrl/kinematics/inverse"

            )
endif()


