from robodk.robolink import *
from robodk.robomath import *
import math
import time
import asyncio


def cal_local_pose(position, rotation):
    local_pose = KUKA_2_Pose(
        [position['x'], position['z'], position['y'],
         rotation['x'], rotation['y'], rotation['z']])
    return local_pose

if __name__ == "__main__":
    RDK = Robolink()
    robot = RDK.Item('KUKA KR 70 R2100-Meltio')

    reference = RDK.Item('Baseline')
    tcp_obj = RDK.Item('tcp_obj')
    bbox = RDK.Item('bbox')

    current_pose = robot.Pose()

    position = {
        'x': current_pose.Pos()[0],
        'y': current_pose.Pos()[1],
        'z': current_pose.Pos()[2]}
    rotation = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    print(position)

    local_pose = cal_local_pose(position, rotation)
    print(local_pose)
    robot.setPose(local_pose)


    print("TCP 각도가 수정되었습니다.")



