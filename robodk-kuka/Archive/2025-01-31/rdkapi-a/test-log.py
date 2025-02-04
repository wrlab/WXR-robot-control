import pandas as pd
from itertools import product, combinations
import numpy as np
from robodk.robomath import *
from robodk.robolink import *

def cal_local_pose(position, rotation):
    # print("cal_local_pose POSITION: ", position)
    local_pose = KUKA_2_Pose(
        [position[0], -position[1], position[2],
         rotation[0], rotation[0], rotation[0]])
    return local_pose

# 로그 파일에서 데이터 로드
#log_file = "C:/GitProjects/robodk-kuka/rdkapi-a/points/pose_log_26.txt"
#log_file = "C:/GitProjects/robodk-kuka/rdkapi-a/points/total_pose_log_25.txt"
log_file = "points/total_pose_log_25.txt"
data = pd.read_csv(log_file)

# RoboDK API 초기화
RDK = Robolink()
robot1 = RDK.Item('KUKA KR 70 R2100-Meltio')
tool1 = robot1.Tool()
turntable1 = RDK.Item('2DOF Turn-table')
reference1 = RDK.Item('Baseline')

rotation = [0, 0, 0]

commands = []

#print(robot1.Pose())

for index, row in data.iterrows():
    position = [row['x'], row['y'], row['z']]
    new_pose = cal_local_pose(position, rotation)
    commands.append(new_pose)
    #print(new_pose)
    robot1.MoveJ(new_pose)


# 버퍼에 저장된 모든 명령어를 순차적으로 실행
# for command in commands:
#     robot1.MoveJ(command)


print("모든 moveJ 명령어가 실행되었습니다.")