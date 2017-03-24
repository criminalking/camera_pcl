# Pose Recognition

## What is Pose Recognition?

Detect which pose people do (e.g. stand, stretch out right hand, stretch out left hand...), use some templates to match test images from binocular cameras.

## Usage
* roscore
* source XXX/setup.bash (**if you use zsh or something else, please change bash to zsh**)
* roslaunch face_detection face_detection_dlib.launch (**open package face_detection**)
* rosrun my_pcl_tutorial PoseRecognition (**open package my_pcl_tutorial**)
* rosrun rviz rviz (**if you want to use rviz to show point clouds**)

## Pre-requisite
* [pcl 1.8](http://pointclouds.org/downloads/)
* [ros Kinetic](http://www.ros.org/)
* [dlib 19.1](https://github.com/davisking/dlib)
* [opencv 2.4.13](http://opencv.org/)

## Configuration
* System: Ubuntu 16.04
* Programming language: c++

## Drawbacks
* Clothes are very important impact elements
* Use an unstable trick to recognize poses in xyz-space

## Thanks
* Date: 31-8-2016
* Author: criminalking
