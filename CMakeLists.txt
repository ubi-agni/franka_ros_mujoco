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
    control_msgs
    tf2_ros
    control_toolbox
    controller_manager
    controller_interface
    hardware_interface
    eigen_conversions
    transmission_interface
    franka_hw
    kdl_parser
    boost_sml
    franka_msgs
    franka_gripper
    franka_example_controllers
    mujoco_ros_control
    pluginlib
    urdf
    joint_limits_interface
)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    angles
    controller_interface
    control_toolbox
    hardware_interface
    transmission_interface
    joint_limits_interface
    eigen_conversions
    urdf
    kdl_parser
    boost_sml
    franka_hw
    franka_msgs
    control_msgs
    std_msgs
    franka_gripper
    franka_example_controllers
    mujoco_ros_control
  INCLUDE_DIRS include
  LIBRARIES
    franka_gripper_sim_mujoco
    franka_hw_sim_mujoco
  DEPENDS
    Franka
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
 ${Franka_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIR}
)

add_library(franka_hw_sim_mujoco
  src/franka_hw_mujoco.cpp
  src/joint.cpp
  src/model_kdl.cpp
  src/controller_verifier.cpp
)

if (Franka_VERSION GREATER_EQUAL 0.9)
  target_compile_definitions(franka_hw_sim_mujoco PUBLIC ENABLE_BASE_ACCELERATION)
endif()

target_link_libraries(franka_hw_sim_mujoco
   ${catkin_LIBRARIES}
   ${Franka_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
 )
 target_include_directories(franka_hw_sim_mujoco SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
 )
 add_dependencies(franka_hw_sim_mujoco
  ${franka_hw_sim_mujoco_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
 )

add_library(franka_gripper_sim_mujoco
  src/franka_gripper_mujoco.cpp
)
target_link_libraries(franka_gripper_sim_mujoco
  ${catkin_LIBRARIES}
)
target_include_directories(franka_gripper_sim_mujoco SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
add_dependencies(franka_gripper_sim_mujoco
  ${franka_hw_sim_mujoco_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(DIRECTORY include/franka_mujoco/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES
    franka_gripper_sim_mujoco_plugin.xml
    franka_hw_sim_mujoco_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS franka_hw_sim_mujoco franka_gripper_sim_mujoco
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
