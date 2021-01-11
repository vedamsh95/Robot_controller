add_library(Ctrl
            sdir_ctrl.cpp
        postypes/Robot.h)

find_package(Python3 COMPONENTS Development NumPy)
target_include_directories(Ctrl PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

add_subdirectory(com)
add_dependencies(Ctrl Com)

add_subdirectory(kinematics/direct)
add_subdirectory(kinematics/inverse)
add_dependencies(Ctrl FWKin IVKin)

add_subdirectory(pathplanner/lin)
add_subdirectory(pathplanner/ptp)
add_subdirectory(pathplanner/splines)
add_dependencies(Ctrl PTP LIN SPLINES)

add_subdirectory(postypes)
add_dependencies(Ctrl PosTypes)

target_link_libraries(Ctrl Com FWKin IVKin PTP LIN SPLINES PosTypes Python3::Python Python3::NumPy)
