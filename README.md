# Franka Panda Robot in MuJoCo with ROS support

This repository is a rewrite of the [franka_gazebo](https://github.com/frankaemika/franka_ros/tree/develop/franka_gazebo) package to work with MuJoCo. Program to setup the MuJoCo simulation (ported from the standard simulation.cc program) is also included.

### TODOS:
- [ ] Refine Model/controller parameters (controller unstable)
- [ ] Fix "TF to MSG: Quaternion Not Properly Normalized" warnings
- [ ] Add disabled contacts
- [ ] Split MujocoSimProxy to a separate package and handle communication via ROS messages
- [x] Make use of initial_joint_positions param to set initial position in MuJoCo
