# Franka Panda Robot in MuJoCo with ROS support

This repository is a rewrite of the [franka_gazebo](https://github.com/frankaemika/franka_ros/tree/develop/franka_gazebo) package to work with MuJoCo. Program to setup the MuJoCo simulation (ported from the standard simulation.cc program) is also included.

### TODOS:
- [ ] Sometimes the control gets unstable. Reducing the simulation speed re-stabilizes the robot, so this might have something to do with to much time passing between clock updates while running at realtime?
- [ ] Add disabled contacts
- [ ] Split MujocoSimProxy to a separate package and handle communication via ROS messages
