import numpy as np
from math import acos, degrees
from scipy.spatial.transform import Rotation as R
from robodk.robomath import *
from robodk.robolink import *


class rdkapiMath:
    def __init__(self, Ts=0.01, kp3=30, kd3=4.2, u3_plus=1.0):
        self.Ts = Ts  # sampling 주기
        self.kp3 = kp3  # 3단계 위치 보정
        self.kd3 = kd3  # 3단계 속도 보정
        self.u3_plus = u3_plus  # 3단계 슬라이딩 모드 스위칭

    def rad2degree(self, rad):
        return rad * 180 / math.pi

    def cal_local_pose(self, position, rotation):
        #print("cal_local_pose POSITION: ", position)
        local_pose = KUKA_2_Pose(
            [1000 * position['x'], -1000 * position['z'], 1000 * position['y'],
             rotation['x'], rotation['y'], rotation['z']])
        return local_pose

    def get_rotation_mat(self, pose):
        rotation_mat = np.array(pose).reshape(4, 4)[:3, :3]
        return rotation_mat

    def cal_new_pose_slidingControl(self, current_pose, position, rotation):
        local_pose = self.cal_local_pose(position, rotation)
        target_position = [local_pose.Pos()[0], -local_pose.Pos()[1], local_pose.Pos()[2]]
        error = np.array(target_position) - np.array(current_pose.Pos())

        norm_error = np.linalg.norm(error)
        if norm_error != 0:
            sliding_term = self.u3_plus * error / norm_error
        else:
            sliding_term = self.u3_plus * error

        velocity_command = self.kp3 * error + sliding_term
        new_position = np.array(current_pose.Pos()) + velocity_command * self.Ts

        x_rotation = rotation.get('x')
        y_rotation = rotation.get('y')
        z_rotation = rotation.get('z')

        x_rot_matrix = rotx(x_rotation)
        z_rot_matrix = rotz(y_rotation)
        y_rot_matrix = roty(-z_rotation)
        rotation_matrix = x_rot_matrix * y_rot_matrix * z_rot_matrix

        new_pose = current_pose
        new_pose.setPos(new_position)
        new_pose = transl(new_pose.Pos()) * rotation_matrix

        return new_pose

    def pose_dif(self, pose1, pose2):
        p1 = Pose_2_KUKA(pose1)
        p2 = Pose_2_KUKA(pose2)
        robomath.distance(p1, p2)

        return robomath.distance(p1, p2)

    def position_dif(self, pose1, pose2):
        p1 = Pose_2_KUKA(pose1)
        p2 = Pose_2_KUKA(pose2)
        pos1 = p1[:3]
        pos2 = p2[:3]
        #print("pos1: ", pos1)
        #print("pos2: ", pos2)
        diff = np.linalg.norm(np.array(pos1) - np.array(pos2))

        return diff

    def rot_dif(self, rot1, rot2):
        q1 = self.euler_to_quaternion(rot1)
        q2 = self.euler_to_quaternion(rot2)
        dot_product = abs(q1[3] * q2[3] + q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2])
        angle_rad = 2 * acos(min(1.0, dot_product))  # acos 값이 1을 넘지 않도록 보정
        return degrees(angle_rad)  # 각도로 변환

    def euler_to_quaternion(self, rotation):
        r = R.from_euler('xyz', [rotation['x'], rotation['y'], rotation['z']], degrees=True)
        return r.as_quat()  # Returns [x, y, z, w]

    def euler_to_matrix(self, rotation):
        r = R.from_euler('xyz', [rotation['x'], rotation['y'], rotation['z']], degrees=True)
        return r.as_matrix()  # Returns 3x3 matrix

    def rotation_matrix_difference(self, R1, R2):
        R_diff = np.dot(np.linalg.inv(R1), R2)
        trace = np.trace(R_diff)
        angle_rad = acos(min(1.0, (trace - 1) / 2))  # acos 값이 1을 넘지 않도록 보정
        return degrees(angle_rad)  # 각도로 변환

    def matrix_to_euler(self, rotation_matrix):
        rotation = R.from_matrix(rotation_matrix)
        euler_angles = rotation.as_euler('xyz', degrees=True)
        #print("Tool Rotation (Euler Angles):", euler_angles)
        return euler_angles

