cmake_minimum_required(VERSION 3.7)
project(mpcc)

set(CMAKE_CXX_STANDARD 14)
# Release build, full optimisation
set(CMAKE_CXX_FLAGS "-O2")

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
        models/dynamic_bicycle.h
        models/dynamic_bicycle.cpp
        params/model_params.h
        params/model_params.cpp)

# For matplotlib
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
target_link_libraries(mpcc ${PYTHON_LIBRARIES})

# For HPIPM and Blasfeo
target_link_libraries(mpcc
        ${CMAKE_SOURCE_DIR}/External/hpipm/lib/lib/libhpipm.a
        ${CMAKE_SOURCE_DIR}/External/blasfeo/lib/lib/libblasfeo.a m)
