from rdk_webSocket_multi import WebSocketCommunication
from robodk.robolink import *
from rdk_config_v4 import Config_server
import asyncio
from asyncio import Event

async def main():
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

    linear_speed = 10
    angular_speed = 180
    joints_speed = 5
    joints_accel = 40

    # 인스턴스 생성 및 초기화
    stop_event = Event()

    ws_comm = WebSocketCommunication(Config_server.HOST, Config_server.PORT, robot1, robot2, tool1, tool2, turntable1, turntable2, RDK)

    # 초기 설정
    robot1.setPoseFrame(reference1)
    robot1.setPoseTool(tool1)
    robot1.setSpeedJoints(joints_speed)

    robot2.setPoseFrame(reference2)
    robot2.setPoseTool(tool2)
    robot2.setSpeedJoints(joints_speed)

    # 웹소켓 서버 태스크 생성
    ws_server_task = asyncio.create_task(ws_comm.start_server())
    #robot_control_task = asyncio.create_task(send2kuka(robot, stop_event))
    #robot_update_task = asyncio.create_task(update_robot_angles(robot1, stop_event))

    # 서버 및 로봇 제어 태스크를 기다림
    await asyncio.gather(ws_server_task)
                         #robot_control_task,
                         #robot_update_task)

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
    joints_speed = 50
    joints_accel = 40

    # 웹소켓 서버 인스턴스 생성
    ws_comm = WebSocketCommunication(Config_server.HOST, Config_server.PORT, robots, tools, ext_tools, RDK)
    print("웹소켓 서버 인스턴스 생성")
    ws_comm.start_server()
    print("웹소켓 서버 비동기적으로 시작")

    # 웹소켓 서버 비동기적으로 시작
    #asyncio.run(main())