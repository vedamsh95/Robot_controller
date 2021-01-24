add_library(LIN
            lin.cpp
        )

if(USE_PYTHON_PLOT)
    target_include_directories(LIN
            PUBLIC
            "${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
            PRIVATE
            ${Python3_INCLUDE_DIRS}
            ${Python3_NumPy_INCLUDE_DIRS}
            )
else()
    target_include_directories(LIN
            PUBLIC
            "${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
            )
endif()