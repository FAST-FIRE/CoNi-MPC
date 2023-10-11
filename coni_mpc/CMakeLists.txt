cmake_minimum_required(VERSION 3.0.2)
project(coni_mpc)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        mavros_msgs
        geometry_msgs
        trajectory_msgs)
find_package(gflags REQUIRED)
if(NOT ${gflags_FOUND})
        message(FATAL_ERROR "Could not find gflags\nPlease use `sudo apt-get install libgflags-dev' to install")
endif()
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)



if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_STANDARD 11)

catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES coni_mpc
        #  CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

# acado model compiling
set(acado_mpc_dep_src
        ${PROJECT_SOURCE_DIR}/acado_model/quadrotor_mpc_codegen/acado_qpoases_interface.cpp
        ${PROJECT_SOURCE_DIR}/acado_model/quadrotor_mpc_codegen/acado_integrator.c
        ${PROJECT_SOURCE_DIR}/acado_model/quadrotor_mpc_codegen/acado_solver.c
        ${PROJECT_SOURCE_DIR}/acado_model/quadrotor_mpc_codegen/acado_auxiliary_functions.c
        ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/Bounds.cpp
        ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/Constraints.cpp
        ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/CyclingManager.cpp
        ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/Indexlist.cpp
        ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/MessageHandling.cpp
        ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/QProblem.cpp
        ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/QProblemB.cpp
        ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/SubjectTo.cpp
        ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/Utils.cpp
        ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
        )

add_library(acado_mpc
        STATIC
        src/acado_mpc/mpc_controller.cpp
        src/acado_mpc/mpc_wrapper.cpp
        src/acado_mpc/mpc_common.cpp
        ${acado_mpc_dep_src})
target_include_directories(acado_mpc PUBLIC
        ${PROJECT_SOURCE_DIR}/acado_model/quadrotor_mpc_codegen
        ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases
        ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/INCLUDE
        ${PROJECT_SOURCE_DIR}/../no_catkin_compilation/acado-stable/external_packages/qpoases/SRC)
target_link_libraries(acado_mpc ${catkin_LIBRARIES} Eigen3::Eigen)


add_library(NumericalCoNiMPC
        STATIC
        src/coni_mpc/num_sim_mpc.cpp
        src/coni_mpc/mpc_base.cpp)
target_include_directories(NumericalCoNiMPC PUBLIC
        ${PROJECT_SOURCE_DIR}/include/coni_mpc)
target_link_libraries(NumericalCoNiMPC
        acado_mpc
        ${catkin_LIBRARIES}
        Eigen3::Eigen)


add_executable(num_sim_non_one_point_node
        src/num_sim/num_sim_non_one_point_node.cpp)
target_link_libraries(num_sim_non_one_point_node
        NumericalCoNiMPC
        gflags
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        Eigen3::Eigen)

