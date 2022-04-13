# Franka Panda Robot in MuJoCo with ROS support

This repository is a rewrite of the [franka_gazebo](https://github.com/frankaemika/franka_ros/tree/develop/franka_gazebo) package to work with MuJoCo. Program to setup the MuJoCo simulation (ported from the standard simulation.cc program) is also included.

### TODOS:
- [ ] Sometimes MuJoCo get's a big sync offset (so far only happened while using moveit to move the robot and only when getting close to the singularities), tries to re-sync, which causes a everything to blow up. Might have something to do with libfranka 0.9 update.
- [ ] Incorporate changes made in franka_ros 0.9.0
- [ ] Add disabled contacts
- [ ] Split MujocoSimProxy to a separate package and handle communication via ROS messages
