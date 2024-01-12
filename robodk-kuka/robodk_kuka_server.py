from robodk_webSocket import WebSocketCommunication
from robodk.robolink import Robolink
from robodk_config import Config
import asyncio

# 로봇 연결
RDK = Robolink()
robot = RDK.Item('KUKA KR 70 R2100')
tool = robot.Tool()
turntable = RDK.Item('2DOF Turn-table')
reference = RDK.Item('Baseline')

# 웹소켓 통신 모듈 인스턴스 생성 및 서버 시작
if __name__ == "__main__":
    ws_comm = WebSocketCommunication(Config.HOST, Config.PORT, robot, tool, turntable, RDK)
    ws_comm.start_server()
