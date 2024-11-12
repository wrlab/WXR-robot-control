from robodk.robolink import *

# RoboDK에 연결
RDK = Robolink()

robot = RDK.Item('KUKA KR 70 R2100-Meltio') # Get robot item.
frame = RDK.Item('Baseline') # Get frame item.
robot.setPoseFrame(frame) # Set the "frame" as the active reference frame.

print("robot joints: ", robot.Joints().list())
current_pose = robot.Pose()
print("robot pose:",  current_pose.Pos())
print(current_pose)

# Pose를 위치와 오일러 각도로 변환합니다.
[x, y, z, rx, ry, rz] = robomath.Pose_2_TxyzRxyz(current_pose)
print('TCP 위치 (mm): X={:.3f}, Y={:.3f}, Z={:.3f}'.format(x, y, z))


#pose = robomath.xyzrpw_2_pose([10, 20, 30, 90, 20, 10]) # Define the XYZ position and RPW angles of the pose you would like the tool to take wrt the frame.
#robot.MoveJ(pose) # Move the robot to "pose".