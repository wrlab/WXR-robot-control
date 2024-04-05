from rdk_webSocket_v2 import WebSocketCommunication
from robodk.robolink import *
from rdk_config_v2 import Config_server
from rdk_config_v2 import Config_host2
import asyncio
from asyncio import Event
from kukaClient import kukaClient


# 24.03.08 기준 완성해야함
# AM 솔루션즈 출장가서 출력된 메시지를 확인하고,
# 해당 포맷에 맞는 메소드 개발할 예정

async def parse_joint_data_corrected(response):
    # 문자열에서 필요하지 않은 초기 부분을 제거
    response = response.split('{')[1]
    # 쉼표로 문자열을 분리하여 각 관절-값 쌍을 얻음
    pairs = response.split(',')
    # 추출된 관절 값들을 저장할 리스트 초기화
    joints = [0.0] * 6  # A1부터 A6까지의 자리를 0.0으로 사전 채움
    # 각 쌍에 대해 반복
    for pair in pairs:
        # 공백으로 각 쌍을 분리하여 관절 이름과 그 값을 분리
        parts = pair.split()
        # 관절 이름이 'A'로 시작하고 1에서 6 사이의 숫자로 끝나는지 확인
        if parts[0].startswith('A') and parts[0][1:].isdigit():
            index = int(parts[0][1:]) - 1  # 관절 번호에 기반한 인덱스 계산
            if 0 <= index < 6:  # 인덱스가 예상 범위 내에 있는지 확인
                # 해당 인덱스에서 값 업데이트
                joints[index] = float(parts[1])
    return joints


async def update_robot_angles(robot, stop_event):
    kuka_client = kukaClient(Config_host2.HOST, Config_host2.PORT)
    print(kuka_client.ip)
    print(kuka_client.port)
    if not kuka_client.can_connect:
        print("KUKA 서버에 연결할 수 없습니다.")
        return
    try:
        count = True
        while not stop_event.is_set():
            # 서버로부터 데이터 요청
            response = kuka_client.read("AXIS_ACT_MEAS", False)
            # response = kuka_client.read("AXIS_ACT", False)
            # response = kuka_client.read("POS_ACT_MES", False)
            # response = kuka_client.read("POS_ACT", False)
            # joints_list = parsing_data(response)
            # joints_list = parse_joint_data_corrected(response)
            joints_list = [0, -90, 80, 0, 0, 0]
            joints_list2 = [0, -90, 70, 0, 0, 0]
            # robot.setJoints(joints_list)
            if response:
                #print(f"서버로부터 받은 관절각도: {response}")
                if robot and count:
                    robot.setJoints(joints_list)
                    print(joints_list)
                    count = False
                else:
                    robot.setJoints(joints_list2)
                    print(joints_list2)
                    count = True

            await asyncio.sleep(0.5)  # 비동기 대기
    finally:
        kuka_client.close()  # 클라이언트 연결 종료


async def main():
    # 로봇 연결
    RDK = Robolink()
    robot = RDK.Item('KUKA KR 70 R2100-Precitec')
    tool = robot.Tool()
    turntable = RDK.Item('2DOF Turn-table')
    reference = RDK.Item('Baseline')

    linear_speed = 10
    angular_speed = 180
    joints_speed = 5
    joints_accel = 40

    # 인스턴스 생성 및 초기화
    stop_event = Event()

    # 초기 설정
    robot.setPoseFrame(reference)
    robot.setPoseTool(tool)
    robot.setSpeedJoints(joints_speed)

    # 웹소켓 서버 태스크 생성
    robot_update_task = asyncio.create_task(update_robot_angles(robot, stop_event))

    # 서버 및 로봇 제어 태스크를 기다림
    await asyncio.gather(  # ws_server_task,
        # robot_control_task,
        robot_update_task)


# 웹소켓 통신 모듈 인스턴스 생성 및 서버 시작
if __name__ == "__main__":
    asyncio.run(main())
