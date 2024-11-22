cmake_minimum_required(VERSION 3.5)
project(sim_raisim_srb VERSION 1.0.0)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3")

if($ENV{USER} STREQUAL "seop")
    set(SIMULATION TRUE) # Chooses to compile the simulation executable or the experiment executable
    add_definitions(-DRUNNING_SIM) # Adds the toe offset to be used in simulation (exp needs no offset)
    set(LL_DIR /home/seop/git_repo_temp/cooperativelocomotion_SRBMPC/lowlevelctrl) # Path to low level files
    set(PL_DIR /home/seop/git_repo_temp/cooperativelocomotion_SRBMPC/plannerSRBMPC) # Path to planner files
else()
    set(SIMULATION FALSE)
endif()


#set(HANKEL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/hankel/) # Path to the hankel matrices (for DeePC)
set(BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin) # Path to the binary folder (for the executables)

# ===== Find Necessary Packages ===== #
find_package(raisim CONFIG REQUIRED HINTS "/home/seop/raisimenv/raisim_build/lib/cmake")
find_package(OGRE CONFIG REQUIRED HINTS "/home/seop/raisimenv/raisim_build/lib/OGRE/cmake")
find_package(raisimOgre CONFIG REQUIRED HINTS "/home/seop/raisimenv/raisim_build/share/raisimOgre/cmake")
#find_package(osqp REQUIRED HINTS "/home/seop/git_repo/osqp/build")

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

# ===== QP SWIFT Library ===== #
add_library(optim_lib
    "${LL_DIR}/optimization/iSWIFT/src/Auxilary.c"
    "${LL_DIR}/optimization/iSWIFT/src/Prime.c"
    "${LL_DIR}/optimization/iSWIFT/src/timer.c"
    "${LL_DIR}/optimization/iSWIFT/ldl/src/ldl.c"
    "${LL_DIR}/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp")
target_include_directories(optim_lib PUBLIC 
    ${EIGEN3_INCLUDE_DIRS}
    ${LL_DIR}/optimization/iSWIFT/include
    ${LL_DIR}/optimization/iSWIFT/ldl/include
    ${LL_DIR}/optimization/iSWIFT/cpp_wrapper)
target_include_directories(optim_lib PRIVATE
    ${LL_DIR}/include)

# ===== Dynamics Library ===== #
# Settings:
set(COMPILE_CORIOLIS false) # Option to compile the coriolis terms. Coriolis is not implemented in code though, may as well not compile.
set(DYN_DIR ${LL_DIR}/dynamics/A1_New_wFoot)

set(dyn_srcs 
    "${DYN_DIR}/A1_D_B_G.cpp"
    "${DYN_DIR}/A1_FK.cpp"
    "${DYN_DIR}/A1_Jaco.cpp"
    "${DYN_DIR}/A1_Jaco_dot.cpp"
    "${LL_DIR}/dynamics/RobotModel.cpp")
if (COMPILE_CORIOLIS)
    set(dyn_srcs ${dyn_srcs} "${DYN_DIR}/A1_C.cpp")
endif()
add_library(dyn_lib ${dyn_srcs})
target_include_directories(dyn_lib PUBLIC
    ${LL_DIR}/dynamics
    ${DYN_DIR}
    ${LL_DIR}/util_include
    ${LL_DIR}/global_include)

# ===== Parameter library ===== #
add_library(params_lib
    "${LL_DIR}/params/Parameters.cpp")
target_include_directories(params_lib PUBLIC
    ${LL_DIR}/params
    ${LL_DIR}/global_include)

# ===== Planning and Contact ===== #
add_library(plan_lib
    "src/ContactEst.cpp"
    "src/MotionPlanner.cpp")
target_include_directories(plan_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${LL_DIR}/util_include
    ${LL_DIR}/global_include)

# ===== Controller Library ===== #
add_definitions(-DUSE_QPSWIFT=1)
add_library(ctrl_lib
    "${LL_DIR}/src/LowLevelCtrl.cpp"
    "${LL_DIR}/src/VirtualConstraints.cpp"
    "${LL_DIR}/src/DataLog.cpp")
target_include_directories(ctrl_lib PUBLIC 
    ${LL_DIR}/include
    ${LL_DIR}/util_include
    ${LL_DIR}/global_include)
target_include_directories(ctrl_lib PRIVATE
    ${LL_DIR}/include)
target_link_libraries(ctrl_lib PUBLIC 
    dyn_lib
    params_lib
    optim_lib)

# ===== Kalman Filter ===== #
add_library(kalman_lib
    "src/kalman.cpp")
target_include_directories(kalman_lib PUBLIC 
    ${LL_DIR}/util_include
    ${CMAKE_CURRENT_SOURCE_DIR}/inc)

# ===== High Level Library ===== #
add_library(srbCoop_lib
     "${PL_DIR}/src/srbcoopModel.cpp")
target_include_directories(srbCoop_lib PUBLIC
      ${PL_DIR}/include
      ${EIGEN3_INCLUDE_DIRS})
add_library(srbmpc_lib
     "${PL_DIR}/src/locomotion_planner.cpp")
target_include_directories(srbmpc_lib PUBLIC
      ${PL_DIR}/include
      ${PL_DIR}/a1_controller
      ${LL_DIR}/optimization/iSWIFT/include
      ${LL_DIR}/optimization/iSWIFT/ldl/include
      ${LL_DIR}/optimization/iSWIFT/cpp_wrapper
      ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(srbmpc_lib PUBLIC
      srbCoop_lib)


# ===== Wrapper ===== #
add_library(wrapper_lib
    "src/LocoWrapper.cpp")
target_include_directories(wrapper_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(wrapper_lib PUBLIC
    ctrl_lib
    plan_lib)

# ===== RaiSim ===== #
if (raisimOgre_FOUND AND SIMULATION)
    add_executable(Run_Raisim driver_scripts/A1_srb_raisim.cpp)
    target_include_directories(Run_Raisim PRIVATE ${LL_DIR}/raisimOgre_include)
    target_include_directories(Run_Raisim PRIVATE ${PL_DIR}/include)
    target_link_libraries(Run_Raisim PRIVATE ${Boost_LIBRARIES} srbmpc_lib wrapper_lib raisim::raisimOgre) #this is for without joystick
    target_compile_definitions(Run_Raisim PUBLIC "-DEXAMPLE_ROBOT_RESOURCE_DIR=${LL_DIR}/rsc/")
    set_target_properties(Run_Raisim PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})
endif()


# EOF
