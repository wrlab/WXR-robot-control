from robodk_webSocket_v2 import WebSocketCommunication
from order2kuka import *
from robodk.robolink import *
from robodk_config import Config
import asyncio
from asyncio import Event


async def main():
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

    # 인스턴스 생성 및 초기화
    stop_event = Event()
    ws_comm = WebSocketCommunication(Config.HOST, Config.PORT, robot, tool, turntable, RDK)

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

    # 초기 설정
    robot.setPoseFrame(reference)
    robot.setPoseTool(tool)
    robot.setSpeedJoints(joints_speed)

    # 웹소켓 서버 태스크 생성
    ws_server_task = asyncio.create_task(ws_comm.start_server())
    robot_control_task = asyncio.create_task(send2kuka(robot, stop_event))

    # 서버 및 로봇 제어 태스크를 기다림
    await asyncio.gather(ws_server_task, robot_control_task)

# 웹소켓 통신 모듈 인스턴스 생성 및 서버 시작
if __name__ == "__main__":
    asyncio.run(main())

    # # 로봇 제어 루프
    # while True:
    #     joints = robot.Joints()
    #     print("Current joint values are:", joints)
    #
    #     robot.MoveJ(joints)
    #     print("Robot MoveJ")
    #     time.sleep(1)
    #
    #     print("Robot...")
    #
    #     # 무한 루프를 방지하기 위한 대기 시간
    #     time.sleep(1)
    #
    #     if isStop:
    #         break
