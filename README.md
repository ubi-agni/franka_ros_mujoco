# Franka Panda Robot in MuJoCo with ROS support

This repository is a rewrite of the [franka_gazebo](https://github.com/frankaemika/franka_ros/tree/develop/franka_gazebo) package that provides a simulated panda hardware interface for [mujoco_ros](https://github.com/DavidPL1/mujoco_ros_pkgs).

# Compatibility notes

This package targets ROS 1 and the hybrid `mujoco_ros_control` package. The hardware simulation plugin exports `franka_mujoco/FrankaHWSim` as a `mujoco_ros::control::RobotHWSim` implementation and uses the current `InitSim`, `ReadSim`, `WriteSim`, and `EStopActive` hooks.

The package still follows the ROS 1 `franka_ros` interfaces:

- `franka_hw/FrankaStateInterface`
- `franka_hw/FrankaModelInterface`
- `franka_control` services and error recovery action
- `franka_gripper` action interfaces for the simulated gripper controller

The launch files pass the current `franka_description` Panda xacro arguments explicitly, including `parent:=world` for Gazebo-style URDF generation. Controller YAML files use `$(arg arm_id)` for Panda joint names, so namespaced arms such as `L_panda` and `R_panda` can share the same controller config.

# License
This work is licensed under the BSD 3-Clause License (see [LICENSE](./LICENSE)).
The franka_gazebo package, which this package is based on, was released under the Apache 2.0 License (see [LICENSE-ORIGINAL](./LICENSE-ORIGINAL)).
