from robodk.robolink import *
from robodk import robomath as rm
import time

# 로봇 연결
RDK = Robolink()
robot = RDK.Item('KUKA KR 70 R2100')
tool = robot.Tool()
turntable = RDK.Item('2DOF Turn-table')
reference = RDK.Item('Baseline')

linear_speed = 10
angular_speed = 180
joints_speed = 5
joints_accel = 40
is_stop = False

if __name__ == "__main__":

    # Connect to the robot using default connetion parameters
    success = robot.Connect()
    print(success)
    status, status_msg = robot.ConnectedState()
    print(status)
    print(status_msg)
    if status != ROBOTCOM_READY:
        # Stop if the connection did not succeed
        raise Exception("Failed to connect: " + status_msg)
    # Set to run the robot commands on the robot
    RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Note: This is set automatically if we use
    # robot.Connect() through the API

    # 초기 설정
    robot.setPoseFrame(reference)
    robot.setPoseTool(tool)
    robot.setSpeedJoints(joints_speed)

    while True:
        joints = robot.Joints()
        print("Current joint values are:", joints)

        robot.MoveJ(joints)
        print("Robot MoveJ")
        time.sleep(1)
        if is_stop:
            break

    RDK.ShowMessage("Done", True)