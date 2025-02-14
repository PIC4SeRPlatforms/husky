husky
=====

## Build Status

[![Formatting (pre-commit)](https://github.com/andreaostuni/husky/actions/workflows/format.yaml/badge.svg?branch=humble-devel)](https://github.com/andreaostuni/husky/actions/workflows/format.yaml?query=branch%3Ahumble-devel)
[![Build and Test](https://github.com/andreaostuni/husky/actions/workflows/build_and_test.yml/badge.svg?branch=humble-devel)](https://github.com/andreaostuni/husky/actions/workflows/build_and_test.yml?query=branch%3Ahumble-devel)

:warning: **Upgrading to ROS2 may not have the same functionally as ROS**: Contact support at support@clearpathrobotics.com.

Common ROS packages for the Clearpath Husky, useable for both simulation and
real robot operation.

 - husky_control : Control configuration
 - husky_description : Robot description (URDF)
 - husky_msgs : Message definitions
 - husky_navigation : Navigation configurations and demos

For Husky instructions and tutorials, please see [Robots/Husky](http://wiki.ros.org/Robots/Husky).

To create a custom Husky description or simulation, please fork [husky_customization](https://github.com/husky/husky_customization).

husky_desktop
=============

Desktop ROS packages for the Clearpath Husky, which may pull in graphical dependencies.

 - husky_viz : Visualization (rviz) configuration and bringup

For Husky instructions and tutorials, please see http://wiki.ros.org/Robots/Husky

husky_robot
===========

Robot ROS packages for the Clearpath Husky, for operating robot hardware.

 - husky_bringup : Bringup launch files and scripts.
 - husky_base : Hardware driver for communicating with the onboard MCU.

For Husky instructions and tutorials, please see http://wiki.ros.org/Robots/Husky

husky_simulator
==============

Simulator ROS packages for the Clearpath Husky.

 - husky_gazebo : Gazebo plugin definitions and extensions to the robot URDF.

For Husky instructions and tutorials, please see http://wiki.ros.org/Robots/Husky
