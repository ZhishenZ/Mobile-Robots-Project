# README #

This is the AR Drone practical repository.

Setup
=====

Ubuntu 18.04 or 20.04
---------------------

Install system dependencies -- this has been done for all the machines in Huxley 202/206:

    sudo apt-get install libsdl1.2-dev libsdl2-dev

Install ROS -- this has been done for all the machines in Huxley 202/206:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt-get update

Then, on Ubuntu 20.04

    sudo apt-get install ros-noetic-desktop-full

... or on Ubuntu 18.04

    sudo apt-get install ros-melodic-desktop-full

Other Systems
-------------

Make sure to have SDL 1.2 and 2 installed. Regarding ROS, please see http://wiki.ros.org/noetic/Installation
    

Setup of ROS Workspace
======================

See Advanced Robotics practical 1 task sheet.
    
