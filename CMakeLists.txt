cmake_minimum_required(VERSION 3.7)
project(alexmpcc)

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

add_executable(alexmpcc
        src/main.cpp
        src/types.cpp
        src/types.h
        src/config.h
        src/Params/params.cpp
        src/Spline/cubic_spline.cpp
        src/Spline/arc_length_spline.cpp
        src/Interfaces/hpipm_interface.cpp
        src/Interfaces/solver_interface.cpp
        src/Constraints/constraints.cpp
        src/Constraints/bounds.cpp
        src/Cost/cost.cpp
        src/Model/model.cpp
        src/Model/integrator.cpp
        src/MPC/mpc.cpp
        src/Params/track.cpp
        src/Params/track.h
        src/Plotting/plotting.cpp
        src/Plotting/plotting.h
        src/ROSnode/fastlapnode.cpp
    	src/ROSnode/fastlapnode.h)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
target_link_libraries(alexmpcc ${PYTHON_LIBRARIES})

target_link_libraries(alexmpcc 
    ${catkin_LIBRARIES}
	${CMAKE_SOURCE_DIR}/src/External/hpipm/lib/lib/libhpipm.a 
	${CMAKE_SOURCE_DIR}/src/External/blasfeo/lib/lib/libblasfeo.a m)
