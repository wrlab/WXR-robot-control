from robodk.robolink import *
from robodk.robomath import *

# RoboDK에 연결
RDK = Robolink()

robot = RDK.Item('KUKA KR 70 R2100-Meltio') # Get robot item.

test_pose = robomath.Pose(1422.500, 0.000, 625.000, 0.000, 0.000, 0.000)
print("test pose: ", test_pose)

positioner = RDK.Item('KUKA KP2-HV500')
test_frame = RDK.Item('test-frame-2')
test_frame_pose = test_frame.Pose()

print("test-frame pose: ", test_frame_pose)

tool = robot.Tool()

reference_poseFrame = robot.PoseFrame()
reference_pose = robot.Pose()
positioner_poseFrame = positioner.PoseFrame()
positioner_pose = positioner.Pose()
print("robot.PoseFrame(): ", reference_poseFrame)
print("robot.Pose(): ", reference_pose)
print("positioner's PoseFrame(): ", positioner_poseFrame)
print("positioner's Pose(): ", positioner_pose)
print("tool: ", tool)

#robot.setPoseFrame(positioner.Pose())
#reference_poseFrame = robot.PoseFrame()

#new_pose = robot.Pose()
#print("tool pose: ", tool)
#print("current pose: ", new_pose)

joints = None

#tool_pose = tool.Pose()
#all_solutions = robot.SolveIK_All(new_pose, tool, positioner_poseFrame)
all_solutions = robot.SolveIK(test_pose, None, tool, test_frame_pose)
#all_solutions = robot.SolveIK_All(test_pose, tool, test_frame_pose)

print("all solutions: ", all_solutions)

if len(all_solutions) == 0:
    print("It can't be reach!")
else:
    for j in all_solutions:
        conf_RLF = robot.JointsConfig(j).list()

        rear = conf_RLF[0]  # 1 if Rear , 0 if Front
        lower = conf_RLF[1]  # 1 if Lower, 0 if Upper (elbow)
        flip = conf_RLF[2]  # 1 if Flip , 0 if Non flip (Flip is usually when Joint 5 is negative)

        # if rear == 0 and lower == 0 and flip == 1:
        if rear == 0 and lower == 0 and flip == 1:
            ik_results = j
            #joints = ik_results[:6]

            joints = [round(value, 3) for value in ik_results[:6]]  # 각 값을 소수점 3자리로 반올림
            print("joint (rounded): ", joints)

            #print("joint: ", joints)
            # robot.setJoints(ik_results[:6])
            # IK 계산이 완료된 후, position과 rotation을 None으로 설정
            break


#frame = RDK.Item('Baseline') # Get frame item.
#robot.setPoseFrame(frame) # Set the "frame" as the active reference frame.
#
# print("robot joints: ", robot.Joints().list())
# current_pose = robot.Pose()
# print("robot pose:",  current_pose.Pos())
# print(current_pose)

# Pose를 위치와 오일러 각도로 변환합니다.
# [x, y, z, rx, ry, rz] = robomath.Pose_2_TxyzRxyz(current_pose)
# print('TCP 위치 (mm): X={:.3f}, Y={:.3f}, Z={:.3f}'.format(x, y, z))


#pose = robomath.xyzrpw_2_pose([10, 20, 30, 90, 20, 10]) # Define the XYZ position and RPW angles of the pose you would like the tool to take wrt the frame.
#robot.MoveJ(pose) # Move the robot to "pose".