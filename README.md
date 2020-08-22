# Hands-free Teleoperation of a Dual-Arm Robot via Real-time 3D Pose Estimation


Install and setup instructions being updated. Discussion of approach & implementation also to come. Check back on Monday!

# Overview

This ROS package is a proof of concept for using ROS and MoveIt to control two robot arms in real-time that mirror human poses captured through a monocular camera feed. 

In practice, manually controlling a robot and deviating it off a pre-programmed path is either often highly imprecise or highly arduous.

Existing human-in-the-loop solutions in industrial manipulators involve the robot driver being sent jogging commands by a handheld controller or other physical input device. Significant data is lost in the process of a human operator translating their movement intention into joystick or button inputs. In addition, the human operator takes on the mental load of extrapolating their actions into 3D space from a third person perspective.

The most precise and instinctive way humans know to interact with the physical world is none other than with their own body. 

Current master-slave implementations within the industry involve devices that are highly specific to a single robot, importable, expensive, and generally unscaleable. Having a cost-free and universal method of control that allows untrained human operators to move the limbs of a robot as if it were their own body may open up new doors within the robotics industry.

## Requirements
* Ubuntu 18.04 LTS
* ROS Melodic Morenia
* Python >= 3.6
* Pytorch >= 1.6.0
* OpenCV >= 4.0

## Set up
Coming soon

## Usage

From Catkin workspace:
```
$ python3 src/handsfree_teleop/pose_estimation/main.py -m human-pose-estimation-3d.pth --video /dev/video0
```
```
$ roslaunch motoman_sda10f_moveit_config moveit_planning_execution.launch sim:=true
```
```
$ roslaunch handsfree_teleop teleop.launch
```
