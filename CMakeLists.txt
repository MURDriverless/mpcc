cmake_minimum_required(VERSION 3.7)
project(mpcc)

set(CMAKE_CXX_STANDARD 14)
# Release build, full optimisation
set(CMAKE_CXX_FLAGS "-O3")
#set(CMAKE_BUILD_TYPE Debug)

# Static link the libraries under External
# The libraries can be obtained by running install.sh
include_directories(.)
include_directories(External/blasfeo/lib/include)
include_directories(External/hpipm/lib/include)
include_directories(External/matplotlib)
include_directories(External/Eigen)
include_directories(External/Json/include)

add_executable(mpcc
        main.cpp
        models/state.h
        models/model_interface.h
        models/dynamic_bicycle_model.h
        models/dynamic_bicycle_model.cpp
        models/model_params.h
        models/model_params.cpp
        splines/binary_search.h
        splines/binary_search.cpp
        splines/cubic_spline.cpp
        splines/cubic_spline.h
        splines/cubic_spline2d.cpp
        splines/cubic_spline2d.h
        costs/cost_types.h
        costs/cost_params.cpp
        costs/cost_params.h
        constraints/constraint_types.h
        costs/contouring_error.cpp
        costs/contouring_error.h
        costs/cost_manager.cpp
        costs/cost_manager.h
        costs/slip_angle.cpp
        costs/slip_angle.h
        constraints/constraint_manager.cpp
        constraints/constraint_manager.h
        constraints/track_constraint.cpp
        constraints/track_constraint.h
        splines/track.h
        constraints/tire_ellipsis_constraint.cpp
        constraints/tire_ellipsis_constraint.h
        constraints/tire_slip_constraint.cpp
        constraints/tire_slip_constraint.h
        bounds/bounds_manager.cpp
        bounds/bounds_manager.h
        mpc/mpc.cpp
        mpc/mpc.h
        mpc/mpc_types.h
        bounds/bounds_types.h
        mpc/mpc_params.cpp
        mpc/mpc_params.h
        models/state.cpp
        solvers/hpipm_interface.cpp
        solvers/hpipm_interface.h
        splines/track.cpp
        plotter/plotter.cpp
        plotter/plotter.h)

# For matplotlib
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
target_link_libraries(mpcc ${PYTHON_LIBRARIES})

# For HPIPM and Blasfeo
target_link_libraries(mpcc
        ${CMAKE_SOURCE_DIR}/External/hpipm/lib/lib/libhpipm.a
        ${CMAKE_SOURCE_DIR}/External/blasfeo/lib/lib/libblasfeo.a m)
