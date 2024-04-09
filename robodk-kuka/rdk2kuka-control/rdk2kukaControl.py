from robodk.robolink import *
from robodk import robomath as rm
import time


if __name__ == "__main__":
    # 로봇 연결
    RDK = Robolink()
    robot = RDK.Item('KUKA KR 70 R2100-Meltio')
    tool = robot.Tool()
    turntable = RDK.Item('2DOF Turn-table')
    reference = RDK.Item('Baseline')

    #current Joints
    current_joints = robot.SimulatorJoints()
    print("Current joint values are:", current_joints)

    linear_speed = 10
    angular_speed = 180
    joints_speed = 5
    joints_accel = 40
    is_stop = False

    # Connect to the robot using default connetion parameters
    success = robot.Connect()
    print("SUCCESS: " + str(success))

    status, status_msg = robot.ConnectedState()
    print("STATUS: " + str(status))
    print("STATUS_MSG: " + status_msg)
    if status != ROBOTCOM_READY:
        # Stop if the connection did not succeed
        raise Exception("Failed to connect: " + status_msg)

    # init
    robot.setPoseFrame(reference)
    robot.setPoseTool(tool)
    robot.setSpeedJoints(joints_speed)

    while True:
        start_time = time.time()
        # Joints가 이전 값과 다른지 확인
        # 이 아래에 작성
        # Joints가 이전 값과 다를 경우
        joints = robot.SimulatorJoints()
        if not current_joints == joints:
            print("Change joint values are:", joints)
            robot.MoveJ(joints)
            print("Robot MoveJ")
            current_joints = joints

            # 종료 시간 측정
            end_time = time.time()
            # 실행 시간 계산
            execution_time = end_time - start_time
            print(f"Execution time: {execution_time} seconds")

            time.sleep(0.004)

        if is_stop:
            break

    RDK.ShowMessage("Done", True)