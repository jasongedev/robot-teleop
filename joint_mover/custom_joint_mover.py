#!/usr/bin/env python
import json
import math
import numpy
import socket
import rospy
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class CustomJointMover:
    def __init__(self):
        self.pub = rospy.Publisher('joint_trajectory_action/goal', FollowJointTrajectoryActionGoal, queue_size=10)

    def generate_trajectory_msg(self, target):
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()

        msg.joint_names = [
            'arm_left_joint_1_s', 
            'arm_left_joint_2_l', 
            'arm_left_joint_3_e', 
            'arm_left_joint_4_u',
            'arm_left_joint_5_r', 
            'arm_left_joint_6_b', 
            'arm_left_joint_7_t',
            'arm_right_joint_1_s', 
            'arm_right_joint_2_l', 
            'arm_right_joint_3_e', 
            'arm_right_joint_4_u', 
            'arm_right_joint_5_r', 
            'arm_right_joint_6_b', 
            'arm_right_joint_7_t',
            'torso_joint_b1',
            'torso_joint_b2'
            ]
        
        point = JointTrajectoryPoint()
        point.positions.append(target[1])
        point.positions.append(target[0])
        point.positions.append(target[3])
        point.positions.append(target[2])
        point.positions.append(0)
        point.positions.append(0)
        point.positions.append(0)
        point.positions.append(target[5])
        point.positions.append(target[4])
        point.positions.append(target[7])
        point.positions.append(target[6])
        point.positions.append(0)
        point.positions.append(0)
        point.positions.append(0)
        point.positions.append(target[8])
        point.positions.append(0)

        for i in range(16):
            point.accelerations.append(1)
            point.velocities.append(5)
            point.time_from_start = rospy.Duration(0)

        msg.points.append(point)

        return msg

    def callback(self, joint_states):
	#TODO: get callbacks from JointState subscription to set start points for motion planning 
        self.current_joint_positions = joint_states.position
        return
    
    def data_loop(self, data):
        target_angles = eval(data.decode())

        trajectory_msg = self.generate_trajectory_msg(target_angles)

        goal_msg = FollowJointTrajectoryGoal()
        goal_msg.trajectory = trajectory_msg

        action_goal_msg = FollowJointTrajectoryActionGoal()
        action_goal_msg.header = trajectory_msg.header
        action_goal_msg.goal = goal_msg
                                    
        self.pub.publish(action_goal_msg)

    def run(self):
        #rospy.Subscriber("joint_states", JointState, self.callback)
        
	#TODO: add bash param in launchfile for setting socket port number 
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(('127.0.0.1', 8082))

        while True:
            self.sock.listen(1)
            connection, client_address = self.sock.accept()
            data = connection.recv(1024)
            if data:
                self.data_loop(data)
                connection.sendall("")
            else:
                self.sock.close()
                break
    
if __name__ == '__main__':
    rospy.init_node('custom_joint_mover', anonymous=True, log_level=rospy.DEBUG)
    rospy.resolve_name
    CustomJointMover().run()
