cmake_minimum_required(VERSION 3.0.2)
project(franka_mujoco)

add_compile_options(-std=c++17)

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

message(STATUS "INSTALLING TO: ${CATKIN_PACKAGE_BIN_DESTINATION}")

find_package(Franka 0.7.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    angles
    # TinyXML

    #From Franka Code
    eigen_conversions
    kdl_parser
    franka_hw
    # franka_msgs
    franka_gripper
    franka_example_controllers
    transmission_interface
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
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
#  ${orocos_kdl_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
 ${Franka_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/main.cpp
    src/mujoco_sim_proxy.cpp
    src/franka_hw_mujoco.cpp
    src/joint.cpp
    src/model_kdl.cpp
    src/uitools.c
)


find_library(GLFW libglfw.so.3)
LINK_DIRECTORIES( ${MUJOCO_HOME}/mujoco/bin )
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   mujoco
   GL
   GLEW
   ${GLFW}
   ${TinyXML_LIBRARIES}
   ${Franka_LIBRARIES}
  #  ${orocos_kdl_LIBRARIES}
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

install(TARGETS ${PROJECT_NAME}_node franka_gripper_mujoco
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
