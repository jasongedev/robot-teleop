import numpy as np
import math

from socket_server import SocketServer

class JointAngleCalculator:
    def estimate_torso_rotation(self, pose):
        l_average_x = (pose[0, 16][0] + pose[0, 18][0] + pose[0, 3][0] + pose[0, 6][0])/4
        r_average_x = (pose[0, 15][0] + pose[0, 17][0] + pose[0, 9][0] + pose[0, 12][0])/4
        l_average_z = (pose[0, 16][1] + pose[0, 18][1] + pose[0, 3][1] + pose[0, 6][1])/4
        r_average_z = (pose[0, 15][1] + pose[0, 17][1] + pose[0, 9][1] + pose[0, 12][1])/4

        if l_average_z > r_average_z:
            return math.atan2(l_average_z , l_average_x + r_average_x) + math.pi / 2 
        else:
            return math.atan2(r_average_z , l_average_x + r_average_x) + math.pi / 2

    def calculate_shoulder_angles(self, shoulder, elbow):
        #Use arctan to find angle between upper arm and vertical spine i.e. armpit angle
        x_distance = abs(shoulder[0] - elbow[0])
        y_distance = abs(shoulder[1] - elbow[1])
        z_distance = abs(shoulder[2] - elbow[2])

        roll = math.atan2(x_distance, y_distance) - math.pi/2
        pitch = math.atan2(z_distance, y_distance) - math.pi/2

        return roll, pitch

    def calculate_elbow_angles(self, shoulder, elbow, wrist):
        #Get dot product of two unit vectors, then apply Law of Cosines
        with np.errstate(divide='ignore',invalid='ignore'):
            upper_arm_vec = (shoulder - elbow) / np.linalg.norm(shoulder - elbow)
            lower_arm_vec = (elbow - wrist) / np.linalg.norm(elbow - wrist)

        dot_product = np.dot(upper_arm_vec, lower_arm_vec)
        axis_angle = np.arccos(np.clip(dot_product, -1.0, 1.0))

        #Get euler angles
        v  = np.cross(upper_arm_vec, lower_arm_vec)
        s, c, t = np.sin(axis_angle), np.cos(axis_angle), 1 - np.cos(axis_angle)
        
        phi = math.atan2(v[1] * s - v[0] * v[2] * t, 1 - (v[1]*v[1] + v[2]*v[2]) * t)
        psi = math.asin(v[0] * v[1] *  t + v[2] * s)
        gamma = math.atan2(v[0] * s - v[1] * v[2] * t, 1 - (v[0]*v[0] + v[2]*v[2]) * t)

        return axis_angle, psi

    def calculate_angles(self, pose):
        if len(pose):
            # x, ,z , y to x, y, z
            l_shoulder_joint = np.array([pose[0, 3][0], pose[0, 3][2], pose[0, 3][1]])
            l_elbow_joint = np.array([pose[0, 4][0], pose[0, 4][2], pose[0, 4][1]])
            l_wrist_joint = np.array([pose[0, 5][0], pose[0, 5][2], pose[0, 5][1]])

            r_shoulder_joint = np.array([pose[0, 9][0], pose[0, 9][2], pose[0, 9][1]])
            r_elbow_joint = np.array([pose[0, 10][0], pose[0, 10][2], pose[0, 10][1]])
            r_wrist_joint = np.array([pose[0, 11][0], pose[0, 11][2], pose[0, 11][1]])

            torso_rotation = self.estimate_torso_rotation(pose)


            l_shoulder_extension, l_shoulder_rotation = self.calculate_shoulder_angles(l_shoulder_joint, l_elbow_joint)
            l_elbow_extension, l_elbow_rotation = self.calculate_elbow_angles(l_shoulder_joint, l_elbow_joint, l_wrist_joint)

            r_shoulder_extension, r_shoulder_rotation = self.calculate_shoulder_angles(r_shoulder_joint, r_elbow_joint)
            r_elbow_extension, r_elbow_rotation = self.calculate_elbow_angles(r_shoulder_joint, r_elbow_joint, r_wrist_joint)


            angles = [0]*9
            angles[0] = l_shoulder_extension 
            angles[1] = l_shoulder_rotation

            angles[2] = -l_elbow_extension
            angles[3] = math.pi/2 -l_elbow_rotation
            if l_shoulder_joint[1]  <  l_wrist_joint[1]:
                angles[3] = angles[3] - math.pi/2

            angles[4] = r_shoulder_extension
            angles[5] = r_shoulder_rotation

            angles[6] = -r_elbow_extension
            angles[7] = math.pi/2 +r_elbow_rotation
            if r_shoulder_joint[1] < r_wrist_joint[1]:
                angles[7] = angles[7] - math.pi/2

            angles[8] = torso_rotation 

            if not np.isnan(angles).any():
                return angles

        return 0