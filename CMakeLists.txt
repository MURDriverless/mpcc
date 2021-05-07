cmake_minimum_required(VERSION 3.7)
project(mpcc)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_msgs
    message_filters
    nav_msgs
    geometry_msgs
    mur_common
    )

add_definitions(-std=c++14)

include_directories(src ${catkin_INCLUDE_DIRS})
include_directories(src/External/blasfeo/lib/include)
include_directories(src/External/hpipm/lib/include)
include_directories(src/External/matplotlib)
include_directories(src/External/Eigen)
include_directories(src/External/Json/include)
catkin_package(CATKIN_DEPENDS)

add_executable(mpcc
        src/main.cpp
        src/models/state.h
        src/models/model_interface.h
        src/models/dynamic_bicycle_model.h
        src/models/dynamic_bicycle_model.cpp
        src/models/model_params.h
        src/models/model_params.cpp
        src/splines/binary_search.h
        src/splines/binary_search.cpp
        src/splines/cubic_spline.cpp
        src/splines/cubic_spline.h
        src/splines/cubic_spline2d.cpp
        src/splines/cubic_spline2d.h
        src/costs/cost_types.h
        src/costs/cost_params.cpp
        src/costs/cost_params.h
        src/constraints/constraint_types.h
        src/costs/contouring_error.cpp
        src/costs/contouring_error.h
        src/costs/cost_manager.cpp
        src/costs/cost_manager.h
        src/costs/slip_angle.cpp
        src/costs/slip_angle.h
        src/constraints/constraint_manager.cpp
        src/constraints/constraint_manager.h
        src/constraints/track_constraint.cpp
        src/constraints/track_constraint.h
        src/splines/track.h
        src/constraints/tire_ellipsis_constraint.cpp
        src/constraints/tire_ellipsis_constraint.h
        src/constraints/tire_slip_constraint.cpp
        src/constraints/tire_slip_constraint.h
        src/bounds/bounds_manager.cpp
        src/bounds/bounds_manager.h
        src/mpc/mpc.cpp
        src/mpc/mpc.h
        src/mpc/mpc_types.h
        src/bounds/bounds_types.h
        src/mpc/mpc_params.cpp
        src/mpc/mpc_params.h
        src/models/state.cpp
        src/solvers/hpipm_interface.cpp
        src/solvers/hpipm_interface.h
        src/splines/track.cpp
	src/ROSnode/fastlapnode.cpp
	src/ROSnode/fastlapnode.h
)

# For matplotlib
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
target_link_libraries(mpcc ${PYTHON_LIBRARIES})

# For HPIPM and Blasfeo
target_link_libraries(mpcc
	${catkin_LIBRARIES}
        ${CMAKE_SOURCE_DIR}/src/External/hpipm/lib/lib/libhpipm.a
        ${CMAKE_SOURCE_DIR}/src/External/blasfeo/lib/lib/libblasfeo.a m)


