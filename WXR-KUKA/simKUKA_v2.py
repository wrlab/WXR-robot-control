from robodk.robomath import *
from robodk.robodialogs import *
from robodk.robofileio import *
from robodk.robolink import *

# Start the RoboDK API
RDK = Robolink()

# Ask the user to pick an SRC file:
rdk_file_path = RDK.getParam("C:/Users/JaeHongYoo/WRL Dropbox/Jaehong Yoo/RoboDK")
# src_file_path = getOpenFileName(rdk_file_path)
src_file_paths = getOpenFileNames(rdk_file_path, "Select SRC Files", "*.src")
if not src_file_paths:
    print("Nothing selected")
    quit()

# 2023.11.08 여러 src 파일 선택 코드
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

    # Ask the user to select a robot (if more than a robot is available)
    robot = RDK.Item('KUKA KR 70 R2100')

    if not robot.Valid():
        raise Exception("Robot not selected or not valid")

    # Get the active reference frame
    frame = RDK.Item('Baseline')

    if not frame.Valid():
        # If there is no active reference frame, use the robot base
        frame = robot.Parent()

    # Get the active tool frame
    tool = RDK.Item('R7')

    # Add a new program
    program = RDK.AddProgram(program_name, robot)

    # Turn off rendering (faster)
    RDK.Render(False)

    # Speed up by not showing the instruction:
    program.ShowInstructions(False)

    # Open the file and iterate through each line
    with open(src_file_path) as f:
        count = 0
        for line in f:
            # Remove empty characters:
            line = line.strip()
            print("Loading line: " + line)

            # Get all the numeric values in order
            values = GetValues(line)

            # Increase the counter
            count = count + 1

            # Update TCP speed (KUKA works in m/s, RoboDK works in m/s)
            # if line.startswith("$VEL.CP"):
            #    program.setSpeed(values[0]*1000)
            #    continue

            # Check operations that involve a pose
            if len(values) < 6:
                print("Warning! Invalid line: " + line)
                continue

            # Check what instruction we need to add:
            if line.startswith("LIN"):
                # 1. BaseFrame을 origin으로 타겟 생성
                target = RDK.AddTarget('T%i' % count, frame)
                # target.setAsJointTarget()
                # Check if we have external axes information, if so, provied it to joints E1 to En
                if len(values) > 6:
                    target.setJoints([0, 0, 0, 0, 0, 0] + values[6:])
                    # values가 external axes를 가질 때, 턴테이블에 대한 움직임 명령 추가

                # 2. 타겟에 values적용
                target.setPose(KUKA_2_Pose(values[:6]))
                # 3. 프로그램에 타겟으로 라인 이동하는 명령 추가
                program.MoveL(target)

            # Check PTP move
            elif line.startswith("PTP"):
                target = RDK.AddTarget('T%i' % count, frame)
                target.setAsJointTarget()
                target.setJoints(values)
                program.MoveJ(target)

            # Set the tool
            elif line.startswith("$TOOL"):
                # pose = KUKA_2_Pose(values[:6])
                # tool = robot.AddTool(pose, "SRC TOOL")
                program.setTool(tool)

            # Set the reference frame
            elif line.startswith("$BASE"):
                # frame = RDK.AddFrame("SRC BASE", robot.Parent())
                # frame.setPose(KUKA_2_Pose(values[:6]))
                robot.setPoseFrame(frame)
                program.setFrame(frame)

    # Hide the targets
    program.ShowTargets(False)

    # Show the instructions
    program.ShowInstructions(False)

RDK.ShowMessage("Done", False)
print("Done")
