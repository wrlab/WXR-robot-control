from robodk.robolink import *

# RoboDK에 연결
RDK = Robolink()

robot = RDK.Item('KUKA KR 70 R2100-Meltio') # Get robot item.
frame = RDK.Item('Baseline') # Get frame item.
robot.setPoseFrame(frame) # Set the "frame" as the active reference frame.

current_pose = robot.Pose()
print(current_pose)
pose = robomath.xyzrpw_2_pose([10, 20, 30, 90, 20, 10]) # Define the XYZ position and RPW angles of the pose you would like the tool to take wrt the frame.
robot.MoveJ(pose) # Move the robot to "pose".