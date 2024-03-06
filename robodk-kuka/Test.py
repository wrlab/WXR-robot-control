from robodk.robolink import Robolink
from robodk import robomath as rm
import time
import math
import asyncio

# 로봇 연결
RDK = Robolink()
robot = RDK.Item('KUKA KR 70 R2100')
tool = robot.Tool()
turntable = RDK.Item('2DOF Turn-table')
reference = RDK.Item('Baseline')

linear_speed = 10
angular_speed = 180
joints_speed = 20
joints_accel = 40

if __name__ == "__main__":
    targetJoints1 = [0, -70, 90, 0 , 0, 0]
    targetJoints2 = [0, -60, 80, 10, 0, 0]

    # x1, y1, z1 = 0, 10, 10
    # x2, y2, z2 = 10, 20, 30
    # x3, y3, z3 = 20, 20, 50
    # targetPose1 = rm.Mat([
    #     [0, 0, 0, x1],  # x 축 방향과 위치
    #     [0, 1, 0, y1],  # y 축 방향과 위치
    #     [0, 0, 1, z1],  # z 축 방향과 위치
    #     [0, 0, 0, 1]  # 동차 좌표 변환 행렬의 마지막 줄
    # ])
    # targetPose2 = rm.Mat([
    #     [1, 0, 0, x2],  # x 축 방향과 위치
    #     [0, 1, 0, y2],  # y 축 방향과 위치
    #     [0, 0, 1, z2],  # z 축 방향과 위치
    #     [0, 0, 0, 1]  # 동차 좌표 변환 행렬의 마지막 줄
    # ])
    # targetPose3 = rm.Mat([
    #     [1, 0, 0, x3],  # x 축 방향과 위치
    #     [0, 1, 0, y3],  # y 축 방향과 위치
    #     [0, 0, 1, z3],  # z 축 방향과 위치
    #     [0, 0, 0, 1]  # 동차 좌표 변환 행렬의 마지막 줄
    # ])
    robot.setPoseFrame(reference)
    robot.setPoseTool(tool)
    robot.setSpeedJoints(joints_speed)

    robot.MoveJ(targetJoints1)
    time.sleep(1.5)
    robot.MoveJ(targetJoints2)
    time.sleep(1.5)

    RDK.ShowMessage("Done", True)

    #program_name = "MoveJ_Test"
    # Add a new program
    #program = RDK.AddProgram(program_name, robot)

    #target1 = RDK.AddTarget("targetJoints1", reference)
    #target2 = RDK.AddTarget("targetJoints2", reference)
    #target3 = RDK.AddTarget("targetPose1", reference)
    #target4 = RDK.AddTarget("targetPose2", reference)
    #target5 = RDK.AddTarget("targetPose3", reference)


