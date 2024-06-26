from wxr2rdk_webSocket_smooth import WebSocketCommunication
from robodk.robolink import *
from config import Config_server
import asyncio
from asyncio import Event

# 웹소켓 통신 모듈 인스턴스 생성 및 서버 시작
if __name__ == "__main__":
    # 로봇 연결
    RDK = Robolink()
    robot1 = RDK.Item('KUKA KR 70 R2100-Meltio')
    tool1 = robot1.Tool()
    turntable1 = RDK.Item('2DOF Turn-table')
    reference1 = RDK.Item('Baseline')

    robot2 = RDK.Item('KUKA KR 70 R2100-Precitec')
    tool2 = robot2.Tool()
    turntable2 = RDK.Item('KUKA KP2-HV500')
    reference2 = RDK.Item('KP2-HV500')

    robots = [robot1, robot2]
    tools = [tool1, tool2]
    ext_tools = [turntable1, turntable2]
    references = [reference1, reference2]

    linear_speed = 10
    angular_speed = 180
    joints_speed = 5
    joints_accel = 40

    # 웹소켓 서버 인스턴스 생성
    ws_comm = WebSocketCommunication(Config_server.HOST, Config_server.PORT, robots, tools, ext_tools, RDK)
    print("웹소켓 서버 인스턴스 생성")
    ws_comm.start_server()
    print("웹소켓 서버 비동기적으로 시작")

    # 웹소켓 서버 비동기적으로 시작
    #asyncio.run(main())