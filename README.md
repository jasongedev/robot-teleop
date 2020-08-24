# Hands-free Teleoperation of a Dual-Arm Robot via Real-time 3D Pose Estimation

This ROS package is a proof of concept for using ROS and MoveIt to control two robot manipulators that mirror human arm poses captured through a monocular camera feed in real-time.

<img src="resources/example1.gif" width=900px>

Install and setup instructions currently being updated. Discussion of approach & implementation also to come, plus some example videos. Check back on Monday!

# Overview

In practice, manually controlling a robot and deviating it off a pre-programmed path is often either highly imprecise or highly arduous.

Existing human-in-the-loop solutions in industrial robots involve the driver being sent jogging commands by a handheld controller or other physical input device. Significant data is lost in the process of a human operator translating their movement intention into joystick or button inputs. In addition, the human operator takes on the significant mental load of extrapolating their action into 3D space from a third person perspective.

The most precise and instinctive method by which humans use to interact with the physical world is by none other than manipulating the limbs of their own body. 

Current master-slave implementations within the industry involve devices that are highly specific to a single robot, importable, expensive, and generally unscaleable. Having a cost-free and universal method of control that allows untrained human operators to move the limbs of a robot as if it were their own body may open up new doors within the robotics industry.

## Requirements
* Ubuntu 18.04 LTS
* ROS Melodic Morenia
* Python >= 3.6
* Pytorch >= 1.6.0
* Numpy >= 1.17
* OpenCV >= 4.0
* CUDA-capable GPU

## Set up
Coming soon

## Usage

From within Catkin workspace:
```
$ roslaunch motoman_sda10f_moveit_config moveit_planning_execution.launch sim:=true
```
```
$ roslaunch handsfree_teleop_launch teleop.launch
```
```
$ python3 src/handsfree_teleop/pose_estimation/main.py --video /dev/video0
```
