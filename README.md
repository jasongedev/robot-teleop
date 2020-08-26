# Hands-free Teleoperation of a Dual-Arm Robot via Real-time 3D Pose Estimation

This ROS package is a proof of concept visualized in RViz for using ROS and MoveIt to control two robot manipulators that mirror human arm poses in real-time captured through a monocular camera feed.

Live demo with commentary: https://youtu.be/JviMLEeE7u8 
<img src="example_gifs/example1.gif" width=1200px>
<img src="example_gifs/example2.gif" width=1200px>

## Table of Contents
- [Overview](#overview)
- [Requirements](#requirements)
- [Setup](#setup)
- [Install](#install)
- [Usage](#usage)
- [How it works](#how-it-works)
- [Discussion](#discussion)

## Overview

In practice, manually controlling a robot and deviating it off a pre-programmed path is either highly imprecise or highly arduous.

Existing human-in-the-loop solutions in commercial robots presently involve the driver being sent jogging commands by a handheld controller or other hardware input device. Significant data is lost in the process of a human operator translating their movement intention into joystick or button inputs. 

In addition, the human operator takes on significant mental load in the form of having to extrapolate their actions into 3D space from a third person perspective. This may lead to slower than desired operation or worse, human error.

The most precise and instinctive method by which we humans use to interact with the physical world around us is by none other than manipulating the limbs of our own body. 

Current master-slave implementations within the industry involve devices that are highly specific to a single robot, importable, expensive, and generally unscaleable. Having a cost-free and universal method of control that allows untrained human operators to move the limbs of a robot as if it were an extension their own body may lead to the opening of new doors within the robotics industry.

## Requirements
* Ubuntu 18.04 LTS
* ROS Melodic Morenia
* Python >= 3.6
* Pytorch >= 1.6
* Numpy >= 1.17
* OpenCV >= 4.0
* CUDA-capable GPU

## Setup
Step 1:
[Create a Catkin workspace](http://wiki.ros.org/melodic/Installation/Source#Create_a_catkin_Workspace)

Step 2:
[Install the ROS-Industrial packages](http://wiki.ros.org/Industrial/Install)

## Install
Build from source into your Catkin workspace:
```
cd catkin_ws
git clone https://github.com/jasongedev/handsfree-teleop/ src/handsfree_teleop
pip3 install -r src/handsfree_teleop/pose_estimation/requirements.txt
rosdep -r install
catkin build -j7
source devel/setup.bash
```

## Usage

From three seperate terminals within your Catkin workspace:
```
$ roslaunch motoman_sda10f_moveit_config moveit_planning_execution.launch sim:=true
```
```
$ roslaunch handsfree_teleop_launch teleop.launch
```
To stream video input from webcam:
```
$ python3 src/handsfree_teleop/pose_estimation/main.py --video /dev/video0
```
OR 

To stream a pre-recorded video:

```
$ python3 src/handsfree_teleop/pose_estimation/main.py --video {VIDEO_FILEPATH.mp4}
```


## How it works

<img src="example_gifs/flowchart.png" width=800px>

## Discussion 
This package is currently modelled to work with the MOTOMAN SDA10F dual arm robot that comes by default with installing the ROS-Industrial packages.

Caution: this package is not ready for production use with a real MOTOMAN SDA10F robot! Attempts to do so will lead to catastrophic results. Or it might not do anything at all as the drivers may refuse to interpret impossible trajectories.

The obvious reason for being so is that, while joint position limits are being accurately simulated, movement is currently too jittery and instantaneous as a result of me as of yet being unable to simulate the velocity and acceleration limits of the real servos in a MOTOMAN SDA10F.

I don’t think temporal smoothing is necessary on the model-level as the movement planner should be more than capable in practice of smoothing the trajectory from one frame to the next without excessive wear on the servos. Instantaneous change of direction might easily be limited with trajectory splicing or by lowering the acceleration values.

I think the necessary information for simulating acceleration and velocity is contained within the simulated drivers and URDF file that come packaged with the MOTOMAN SDA10F config in ROS-Industrial. The next thing I’ll have to try is to see if it might have something to do with setting a start-position for the joint trajectory goal alongside the end-position that it currently snaps to, as well as declaring a duration for the time_from_start variable in the FollowTrajectoryActionGoal message. 

As for applying this package to models beyond the MOTOMAN SDA10F, the core principle behind translating XYZ human skeleton joint coordinates to Euler angles and in turn, robot joint states, is translatable to any singular or dual industrial manipulators corresponding to either or both human arms. 

All it would take is changing the joint names and the order of appended joint positions contained within the JointTrajectoryActionGoal message that is published to the joint_trajectory_action topic by the custom_joint_mover node. 

