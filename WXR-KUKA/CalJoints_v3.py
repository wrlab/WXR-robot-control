from robodk.robomath import *
import robodk.robolink as robolink

# RoboDK API와의 연결 초기화
RDK = robolink.Robolink()

# 첫 번째 로봇을 선택 (로봇 스테이션에 로봇이 이미 있어야 함)
robot = RDK.Item('KUKA KR 70 R2100', robolink.ITEM_TYPE_ROBOT)

tool = robot.Tool()
#tool = RDK.Item('R7', robolink.ITEM_TYPE_TOOL)
#tool_pose = tool.PoseTool()

baseline = RDK.Item('Baseline')

reference_pose = robot.PoseFrame()
baseline_pose = baseline.Pose()

# 명시적으로 pose를 넣어주는 방법!
#target_position = [0.00, -4.757, 40.00]
#target_orientation = [0.00, 0.00, -2.92]
target_position = [1533.696, 36.698, 691.154]
target_orientation = [-0.16, 0.00, 87.156]
target_list = [1533.696, 36.698, 691.154, -0.16, 0.00, 87.156]
# rotx, y, z를 이용하기 위해서 target_orientation의 값을 라디안으로 변환해줘야 한다!
radian_x = target_orientation[0] * (math.pi / 180)
radian_y = target_orientation[1] * (math.pi / 180)
radian_z = target_orientation[2] * (math.pi / 180)

target_pose = transl(target_position) * rotx(radian_x) * roty(radian_y) * rotz(radian_z)
target_str = str(target_pose)
RDK.ShowMessage('target_pose: ' + target_str, True)

frame = RDK.Item('KUKA KR 70 R2100 Base')
target = RDK.AddTarget('Test Target', frame)
#target.setPose(KUKA_2_Pose(target_list))
target.setPose(target_pose)

# 포즈로부터 관절값 계산
#joints = robot.SolveIK(pose, None, tool_pose, reference_pose)
#joints = robot.SolveIK(target_pose, None, tool_pose, baseline_pose)
#joints_str= str(joints)
#RDK.ShowMessage('SolveIK with pose: ' + joints_str, True)

#joints_now = robot.Joints()
#joints_now_str = str(joints_now)
#RDK.ShowMessage('Current joints: ' + joints_now_str, True)

#robot_position = robot.SolveFK(joints_now, tool_pose, reference_pose)
#position_str = str(robot_position)
#RDK.ShowMessage('robot flange pose with SolveFK: ' + position_str, True)

#ik_joints = robot.SolveIK(robot_position, tool_pose, reference_pose)
#ik_str = str(ik_joints)
#RDK.ShowMessage('ik_joints: ' + ik_str, True)

all_solutions = robot.SolveIK_All(target_pose, tool)
joints_list = []
# Iterate through each solution
for j in all_solutions:
    # Retrieve flags as a list for each solution
    conf_RLF = robot.JointsConfig(j).list()

    # Breakdown of flags:
    rear  = conf_RLF[0] # 1 if Rear , 0 if Front
    lower = conf_RLF[1] # 1 if Lower, 0 if Upper (elbow)
    flip  = conf_RLF[2] # 1 if Flip , 0 if Non flip (Flip is usually when Joint 5 is negative)

    # Look for a solution with Front and Elbow up configuration
    #if conf_RLF[0:2] == [0,0]:
    if rear == 0 and lower == 0 and flip == 0:
        print("Solution found!")
        joints_sol = j
        joints_sol_str = str(joints_sol)
        RDK.ShowMessage('SolveIK_All: ' + joints_sol_str, True)
        robot.setJoints(joints_sol[:6])
        break

