# Hands-free Teleoperation of a Dual-Arm Robot via Real-time 3D Pose Estimation

This ROS package is a proof of concept visualized in RViz for using ROS and MoveIt to control two robot manipulators that mirror human arm poses in real-time captured through a monocular camera feed.

<img src="example_gifs/example1.gif" width=1200px>
<img src="example_gifs/example2.gif" width=1200px>

# Overview

In practice, manually controlling a robot and deviating it off a pre-programmed path is either highly imprecise or highly arduous.

Existing human-in-the-loop solutions in industrial robots involve the driver being sent jogging commands by a handheld controller or other hardware input device. Significant data is lost in the process of a human operator translating their movement intention into joystick or button inputs. In addition, the human operator takes on significant mental load by having to extrapolate their actions into 3D space from a third person perspective.

The most precise and instinctive method by which we humans use to interact with the physical world around us is by none other than manipulating the limbs of our own body. 

Current master-slave implementations within the industry involve devices that are highly specific to a single robot, importable, expensive, and generally unscaleable. Having a cost-free and universal method of control that allows untrained human operators to move the limbs of a robot as if it were their own body may lead to the opening of new doors within the robotics industry.

## Requirements
* Ubuntu 18.04 LTS
* ROS Melodic Morenia
* Python >= 3.6
* Pytorch >= 1.6
* Numpy >= 1.17
* OpenCV >= 4.0
* CUDA-capable GPU

## Set up
Step 1:
[Create a Catkin workspace](http://wiki.ros.org/melodic/Installation/Source#Create_a_catkin_Workspace)

Step 2:
[Install the ROS-Industrial packages](http://wiki.ros.org/Industrial/Install)

Step 3:
Install this repo into your Catkin workspace:
```
git clone "https://github.com/jasongedev/handsfree-teleop/" src
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
