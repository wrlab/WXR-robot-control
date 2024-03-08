from robodk_webSocket_v2 import WebSocketCommunication
from order2kuka import *
from robodk.robolink import *
from robodk_config import Config
import asyncio
from asyncio import Event
from kukaClient import  kukaClient

async def update_robot_angles(robot, stop_event):
    kuka_client = kukaClient(Config.HOST, Config.PORT)
    if not kuka_client.can_connect:
        print("KUKA 서버에 연결할 수 없습니다.")
        return
    try:
        while not stop_event.is_set():
            response = kuka_client.read("AXIS_ACT", debug=False)
            if response:
                print(f"서버로부터 받은 관절각도: {response}")
                angles = [float(angle) for angle in response.split(',')]  # 응답을 파싱하여 각도 리스트로 변환
                robot.setJoints(angles)  # 로봇의 관절각도를 설정
            await asyncio.sleep(0.1)  # 비동기 대기
    finally:
        kuka_client.close()  # 클라이언트 연결 종료

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
    print("성공할 경우: True, 실패할 경우: False")
    print(success)
    status, status_msg = robot.ConnectedState()
    print("ConnectedState: ")
    print(status)
    print(status_msg)
    if status != ROBOTCOM_READY:
        # Stop if the connection did not succeed
        raise Exception("Failed to connect: " + status_msg)
    # Set to run the robot commands on the robot
    print("로봇과의 연결이 성공적으로 이루어졌습니다. RUNMODE_RUN_ROBOT을 설정합니다.")
    RDK.setRunMode(RUNMODE_RUN_ROBOT)


    # 초기 설정
    robot.setPoseFrame(reference)
    robot.setPoseTool(tool)
    robot.setSpeedJoints(joints_speed)

    # 웹소켓 서버 태스크 생성
    ws_server_task = asyncio.create_task(ws_comm.start_server())
    #robot_control_task = asyncio.create_task(send2kuka(robot, stop_event))
    robot_update_task = asyncio.create_task(update_robot_angles(robot, stop_event))

    # 서버 및 로봇 제어 태스크를 기다림
    await asyncio.gather(ws_server_task,
                         #robot_control_task,
                         robot_update_task)

# 웹소켓 통신 모듈 인스턴스 생성 및 서버 시작
if __name__ == "__main__":
    asyncio.run(main())