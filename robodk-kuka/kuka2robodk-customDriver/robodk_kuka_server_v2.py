from robodk_webSocket_v2 import WebSocketCommunication
from robodk.robolink import *
from robodk_config_v2 import Config_server
from robodk_config_v2 import Config_host
import asyncio
from asyncio import Event
from kukaClient import kukaClient

# 24.03.08 기준 완성해야함
# AM 솔루션즈 출장가서 출력된 메시지를 확인하고,
# 해당 포맷에 맞는 메소드 개발할 예정
async def parsing_data(response):
    # 파싱할 데이터가 저장될 데이터
    parsed_data = None

    return parsed_data

async def update_robot_angles(robot, stop_event):
    kuka_client = kukaClient(Config_host.HOST, Config_host.PORT)
    print(kuka_client.ip)
    print(kuka_client.port)
    if not kuka_client.can_connect:
        print("KUKA 서버에 연결할 수 없습니다.")
        return
    try:
        while not stop_event.is_set():
            # 서버로부터 데이터 요청
            response = kuka_client.read("AXIS_ACT", False)
            #kuka_client.read("AXIS_ACT", False)
            print(f"서버로부터 받은 관절각도: {response}")
            #joints_list = parsing_data(response)
            #robot.setJoints(joints_list)
            joints_list = [0, -90, 90, 0, 0, 0]
            if robot:
                robot.setJoints(joints_list)

            #if response:
            #    print(f"서버로부터 받은 관절각도: {response}")
                #angles = [float(angle) for angle in response.split(',')]  # 응답을 파싱하여 각도 리스트로 변환
                #robot.setJoints(angles)  # 로봇의 관절각도를 설정
            await asyncio.sleep(0.5) # 비동기 대기
    finally:
        kuka_client.close()  # 클라이언트 연결 종료

async def main():
    # 로봇 연결
    RDK = Robolink()
    robot = RDK.Item('KUKA KR 70 R2100-Meltio')
    tool = robot.Tool()
    turntable = RDK.Item('2DOF Turn-table')
    reference = RDK.Item('Baseline')

    #robot.setJoints(Joints)

    linear_speed = 10
    angular_speed = 180
    joints_speed = 5
    joints_accel = 40

    # 인스턴스 생성 및 초기화
    stop_event = Event()

    ws_comm = WebSocketCommunication(Config_server.HOST, Config_server.PORT, robot, tool, turntable, RDK)

    # # Connect to the robot using default connetion parameters
    # success = robot.Connect()
    # print("성공할 경우: True, 실패할 경우: False")
    # print(success)
    # status, status_msg = robot.ConnectedState()
    # print("ConnectedState: ")
    # print(status)
    # print(status_msg)
    # if status != ROBOTCOM_READY:
    #     # Stop if the connection did not succeed
    #     raise Exception("Failed to connect: " + status_msg)
    # # Set to run the robot commands on the robot
    # print("로봇과의 연결이 성공적으로 이루어졌습니다. RUNMODE_RUN_ROBOT을 설정합니다.")
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)

    # 초기 설정
    robot.setPoseFrame(reference)
    robot.setPoseTool(tool)
    robot.setSpeedJoints(joints_speed)

    # 웹소켓 서버 태스크 생성
    #ws_server_task = asyncio.create_task(ws_comm.start_server())
    #robot_control_task = asyncio.create_task(send2kuka(robot, stop_event))
    robot_update_task = asyncio.create_task(update_robot_angles(robot, stop_event))

    # 서버 및 로봇 제어 태스크를 기다림
    await asyncio.gather(#ws_server_task,
                         #robot_control_task,
                         robot_update_task)

# 웹소켓 통신 모듈 인스턴스 생성 및 서버 시작
if __name__ == "__main__":
    asyncio.run(main())