rgbdtools
===================================

Ivan Dryanovski  
<ivan.dryanovski@gmail.com>

Copyright (C) 2013, City University of New York  
CCNY Robotics Lab  
<http://robotics.ccny.cuny.edu/>
 
Overview
-----------------------------------

The stack contains tools for visual odometry and mapping using RGB-D cameras. 

For ROS applications built using this library, see [ccny_rgbd_tools](https://github.com/ccny-ros-pkg/ccny_rgbd_tools.git).

This code is at an experimental stage, and licensed under the GPLv3 license.

Installing
-----------------------------------

If you are using this library as part of ROS, then refer to the 
[ccny_rgbd_tools](https://github.com/ccny-ros-pkg/ccny_rgbd_tools.git) 
installation instructions. You do not need to install anything manually.

If you would like to build this library stand-alone, then follow these
steps:

If you don't have git installed, do so:

    sudo apt-get install git-core

Download the stack from our repository:

    git clone https://github.com/ccny-ros-pkg/rgbdtools.git

Install the required dependencies:

 * [g2o](https://github.com/RainerKuemmerle/g2o)
 
If you are using ROS, you use the `libg2o` deb package.

Next, configure and build:

    mkdir build
    cd build
    cmake ..
    make

References
-----------------------------------

If you use this system in your reasearch, please cite the following paper:

Ivan Dryanovski, Roberto G. Valenti, Jizhong Xiao. 
*Fast Visual Odometry and Mapping from RGB-D Data*. 
2013 International Conference on Robotics and Automation (ICRA2013).

More info
-----------------------------------

Videos:
 * Visual odometry & 3D mapping: http://youtu.be/YE9eKgek5pI
 * Feature viewer: http://youtu.be/kNkrPuBu8JA
