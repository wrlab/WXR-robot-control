import json
from rdkapi_task import WebSocketCommunication
from robodk.robolink import *
from config import Config_server
import asyncio
import uvloop

asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

def load_profile(filepath):
    with open(filepath, 'r') as file:
        return json.load(file)


# 웹소켓 통신 모듈 인스턴스 생성 및 서버 시작
if __name__ == "__main__":
    # load profile
    #profile_path = "profile/robodk-3.1-profile.json"
    profile_path = "profile/robodk-positioner-test-v3-profile.json"
    profile_data = load_profile(profile_path)

    # 로봇 연결
    RDK = Robolink()

    # initialize robots, tools, ext_tools, and references
    robots = []
    tools = []
    ext_tools = []
    references = []

    for robot_data in profile_data['robots']:
        robot = RDK.Item(robot_data['name'])
        tool = robot.Tool() if robot_data['tool'] == "" else RDK.Item(robot_data['tool'])
        ext_tool = RDK.Item(robot_data['ext_tool'])
        reference = RDK.Item(robot_data['reference'])

        robots.append(robot)
        tools.append(tool)
        ext_tools.append(ext_tool)
        references.append(reference)

    # 웹소켓 서버 인스턴스 생성
    mode_input = input("Select operating mode:\n1) Simulation\n2) Real\nEnter your choice: ")
    ws_comm = WebSocketCommunication(
        Config_server.HOST,
        Config_server.PORT,
        robots, tools,
        ext_tools,
        RDK,
        run_mode=mode_input)
    print("웹소켓 서버 인스턴스 생성")
    ws_comm.start_server()