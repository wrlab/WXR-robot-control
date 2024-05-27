from robodk.robomath import *
from robodk.robodialogs import *
from robodk.robofileio import *
from robodk.robolink import *

# 24.04 SIMTOS 전시 컨테츠 용도로 제작한 RoboDK 시뮬레이션

RDK = Robolink()
robot1 = RDK.Item('KUKA KR 70 R2100 Meltio')
robot2 = RDK.Item('KUKA KR 70 R2100 Precitec')

# reference Frame
meltio_frame = RDK.Item('KUKA KR 70 R2100 Base-Meltio')
precitec_frame = RDK.Item('KUKA KR 70 R2100 Base-precitec')

# Target들을 넣을 frame 생성
target_frame = RDK.Item('TargetFrame')
target_frame2 = RDK.Item('TargetFrame2')

# Program 생성
program_name = 'Program_Meltio'
program2_name = 'Program_Precitec'
program1 = RDK.AddProgram(program_name, robot1)
program2 = RDK.AddProgram(program2_name, robot2)

program1.setFrame(meltio_frame)
program2.setFrame(precitec_frame)

program1.ShowInstructions(True)
program2.ShowInstructions(True)

radius = 300
height = 100

for z in range(height + 1):
    targets = [
        (1600, radius, z, 0, 0, 0),
        (1600 -radius, 0, z, 0, 0, 0),
        (1600, -radius, z, 0, 0, 0)
    ]

    target1_name = f"Target_{z * 3 + 0 + 1}"
    target2_name = f"Target_{z * 3 + 1 + 1}"
    target3_name = f"Target_{z * 3 + 2 + 1}"
    target1_item = RDK.AddTarget(target1_name, target_frame, robot1)
    target2_item = RDK.AddTarget(target2_name, target_frame, robot1)
    target3_item = RDK.AddTarget(target3_name, target_frame, robot1)

    # 3개 타겟 묶음이 홀수로 시작할때
    if z % 2 == 1:
        print(z % 2)
        target1_item.setPose(KUKA_2_Pose(targets[0]))
        target2_item.setPose(KUKA_2_Pose(targets[1]))
        target3_item.setPose(KUKA_2_Pose(targets[2]))

    # 3개 타겟 묶음이 짝수로 시작할때
    elif z % 2 == 0:
        print(z % 2)
        target1_item.setPose(KUKA_2_Pose(targets[2]))
        target2_item.setPose(KUKA_2_Pose(targets[1]))
        target3_item.setPose(KUKA_2_Pose(targets[0]))

    program1.MoveJ(target1_item)
    program1.MoveC(target2_item, target3_item)

for z in range(height + 1):
    targets = [
        (1600, radius, z, 0, 0, 0),
        (1600 -radius, 0, z, 0, 0, 0),
        (1600, -radius, z, 0, 0, 0)
    ]

    target1_name = f"Target_a{z * 3 + 0 + 1}"
    target2_name = f"Target_a{z * 3 + 1 + 1}"
    target3_name = f"Target_a{z * 3 + 2 + 1}"
    target1_item = RDK.AddTarget(target1_name, target_frame2, robot2)
    target2_item = RDK.AddTarget(target2_name, target_frame2, robot2)
    target3_item = RDK.AddTarget(target3_name, target_frame2, robot2)

    # 3개 타겟 묶음이 홀수로 시작할때
    if z % 2 == 1:
        print(z % 2)
        target1_item.setPose(KUKA_2_Pose(targets[0]))
        target2_item.setPose(KUKA_2_Pose(targets[1]))
        target3_item.setPose(KUKA_2_Pose(targets[2]))

    # 3개 타겟 묶음이 짝수로 시작할때
    elif z % 2 == 0:
        print(z % 2)
        target1_item.setPose(KUKA_2_Pose(targets[2]))
        target2_item.setPose(KUKA_2_Pose(targets[1]))
        target3_item.setPose(KUKA_2_Pose(targets[0]))

    program2.MoveJ(target1_item)
    program2.MoveC(target2_item, target3_item)
