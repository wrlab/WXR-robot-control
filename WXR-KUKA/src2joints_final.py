from robodk.robomath import *
from robodk.robodialogs import *
from robodk.robofileio import *
from robodk.robolink import *
from robodk.robolinkutils import *

# Start the RoboDK API
RDK = Robolink()

# Ask the user to pick an SRC file:
rdk_file_path = RDK.getParam("C:/Users/JaeHongYoo/WRL Dropbox/Jaehong Yoo/RoboDK")
# src_file_path = getOpenFileName(rdk_file_path)
src_file_paths = getOpenFileNames(rdk_file_path, "Select SRC Files", "*.src")
if not src_file_paths:
    print("Nothing selected")
    quit()

# Process each selected SRC file
for src_file_path in src_file_paths:
    # Get the program name form the file path
    program_name = getFileName(src_file_path)
    program_name = program_name.replace('_', '').replace('(', '').replace(')', '')
    print("Loading program: " + program_name)

    # Check if the file extension is .src
    if not src_file_path.lower().endswith(".src"):
        print("Invalid file selected: " + src_file_path)
        continue

    def GetValues(line):
        "" "Get all the numeric values from a line"""
        line = line.replace(",", " ")
        line = line.replace("}", " ")
        values = line.split(" ")

        list_values = []
        for value in values:
            try:
                value = float(value)
            except:
                continue

            list_values.append(value)

        print("list_values: ")
        print(list_values[:6])
        return list_values

    # Open the file and iterate through each line
    with open(src_file_path) as f:
        count = 0
        for line in f:
            # Remove empty characters:
            line = line.strip()
            print("Loading line: " + line)

            # Get all the numeric values in order
            values = GetValues(line)
            values_str = str(values)

            # Increase the counter
            count = count + 1

            # Check operations that involve a pose
            if len(values) < 6:
                print("Warning! Invalid line: " + line)
                continue

            # Check what instruction we need to add:
            if line.startswith("LIN"):
                # Ask the user to select a robot (if more than a robot is available)
                robot = RDK.Item('KUKA KR 70 R2100', ITEM_TYPE_ROBOT)
                tool = robot.Tool()

                # 턴테이블 불러오기 및 회전 적용
                turntable = RDK.Item('2DOF Turn-table')
                turntable_joints = [values[6], values[7]]
                turntable.setJoints(turntable_joints)
                #RDK.ShowMessage('Turn-table has changed!', True)

                # 턴테이블 위에 존재하는 Baseline Frame 불러오기 및 전역좌표 계산
                reference = RDK.Item('Baseline')
                reference_pose = reference.PoseAbs()
                str_reference = str(reference_pose)
                #RDK.ShowMessage('reference_pose: ' + str_reference, True)

                # KUKA 기준 TCP 좌표
                kuka_pose = KUKA_2_Pose(values[:6])
                str_kuka_pose = str(kuka_pose)
                #RDK.ShowMessage('kuka_pose: ' + str_kuka_pose, True)

                # 지역 좌표계의 TCP좌표를 전역 좌표계로 변환
                final_pose = reference_pose * kuka_pose
                #final_pose = reference_pose * target_pos
                #final_pose = reference_pose
                str_final_pose = str(final_pose)
                #RDK.ShowMessage('final_pose: ' + str_final_pose, True)

                # TCP좌표를 로봇에 대한 지역 좌표로 변환
                robot_poseAbs = robot.PoseAbs()
                local_pose = invH(robot_poseAbs) * final_pose
                str_local_pose = str(local_pose)
                #RDK.ShowMessage('local_pose: ' + str_local_pose, True)

                all_solutions = robot.SolveIK_All(local_pose, tool)
                joints_list = []
                # Iterate through each solution
                for j in all_solutions:
                    # Retrieve flags as a list for each solution
                    conf_RLF = robot.JointsConfig(j).list()

                    # Breakdown of flags:
                    rear = conf_RLF[0]  # 1 if Rear , 0 if Front
                    lower = conf_RLF[1]  # 1 if Lower, 0 if Upper (elbow)
                    flip = conf_RLF[2]  # 1 if Flip , 0 if Non flip (Flip is usually when Joint 5 is negative)

                    # Look for a solution with Front and Elbow up configuration
                    # if conf_RLF[0:2] == [0,0]:
                    if rear == 0 and lower == 0 and flip == 0:
                        print("Solution found!")
                        joints_sol = j
                        joints_sol_str = str(joints_sol)
                        robot.setJoints(joints_sol[:6])
                        #RDK.ShowMessage('SolveIK_All', True)
                        pause(0.01)
                        break

RDK.ShowMessage("Done", False)