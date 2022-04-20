cmake_minimum_required(VERSION 3.4)
project(franka_mujoco)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    angles
    std_msgs
    tf2_ros
    control_toolbox
    controller_manager
    controller_interface
    hardware_interface
    cmake_modules

    # From Franka Code
    eigen_conversions
    transmission_interface
    franka_hw
    kdl_parser
    # franka_msgs
    franka_gripper
    franka_example_controllers

)

find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    angles
    controller_manager
    controller_interface
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    eigen_conversions
    urdf
    kdl_parser
    franka_hw
    # franka_msgs
    franka_gripper
    franka_example_controllers
  INCLUDE_DIRS include
  LIBRARIES
    franka_gripper_mujoco
  DEPENDS
    Franka
    # orocos_kdl
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
 ${Franka_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIR}
)

add_library(franka_hw_mujoco
  src/mujoco_sim_proxy.cpp
  src/franka_hw_mujoco.cpp
  src/joint.cpp
  src/model_kdl.cpp
  src/uitools.c
  src/controller_verifier.cpp
)

if (Franka_VERSION GREATER_EQUAL 0.9)
  target_compile_definitions(franka_hw_mujoco PUBLIC ENABLE_BASE_ACCELERATION)
endif()

find_library(GLFW libglfw.so.3)
LINK_DIRECTORIES( ${MUJOCO_HOME}/mujoco/bin )
target_link_libraries(franka_hw_mujoco
   ${catkin_LIBRARIES}
   mujoco
   GL
   GLEW
   ${GLFW}
   ${Franka_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
 )
 target_include_directories(franka_hw_mujoco SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
 )
 add_dependencies(franka_hw_mujoco
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
 )

add_library(franka_gripper_mujoco
  src/franka_gripper_mujoco.cpp
)
target_link_libraries(franka_gripper_mujoco
  ${catkin_LIBRARIES}
)
target_include_directories(franka_gripper_mujoco SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
add_dependencies(franka_gripper_mujoco
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  franka_hw_mujoco
)


#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES franka_gripper_mujoco_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS franka_hw_mujoco ${PROJECT_NAME}_node franka_gripper_mujoco
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY assets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/delayed_controller_spawner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_franka_mujoco.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
