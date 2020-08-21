# Hands-free Teleoperation of a Dual-Arm Robot via Real-time 3D Pose Estimation


Docs and build instructions being updated. Discussion of implementation also to come. Check back on Monday!

# Overview

This package is a proof of concept for using ROS and MoveIt to control two robot arms in real-time that mirror human poses captured through a monocular camera feed. 

In practice, manually controlling a robot and making it deviate off a pre-programmed path is either highly imprecise or highly arduous.

Existing human-in-the-loop solutions in industrial robots involve the robot driver being sent jogging commands by a handheld controller or other physical input device. Significant data is lost throughout the process of a human operator translating their movement intention into joystick or button inputs. In addition, the human operator takes on the mental load of interpreting their actions in 3D space from a third person perspective.

The most precise and instinctive way humans know to interact with the physical world is none other than with their own body. 

Current master-slave implementations within the industry involve devices that are highly specific to a single robot, importable, expensive, and generally unscaleable. Having a cost-free and universal method of control that allows untrained human operators to move the limbs of a robot as if it were their own body may open up new doors within the robotics industry.
