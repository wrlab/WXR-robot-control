from robodk.robolink import *
from rdk_config import Config_host
import asyncio
from asyncio import Event
from proxyClient import kukaClient


async def parse_joint_data_corrected(response):
    # 문자열에서 필요하지 않은 초기 부분을 제거
    key, value = str(response).split(":")
    key = key[1:].strip()
    value = value[:-2].strip()
    joints = {k: float(v) for k, v in [j.split() for j in value.split(",")]}
    return list(joints.values())


async def update_robot_angles(robot, stop_event):
    kuka_client = None
    if robot.Name() == 'KUKA KR 70 R2100-Meltio':
        kuka_client = kukaClient(Config_host.HOST, Config_host.PORT)
    print(f"KUKA client for {robot.Name()} IP: {kuka_client.ip}, Port: {kuka_client.port}")
    #print("KUKA client IP: " + str(kuka_client.ip))
    #print("KUKA client Port: " + str(kuka_client.port))
    if not kuka_client.can_connect:
        print("KUKA 서버에 연결할 수 없습니다.")
        return
    try:
        while not stop_event.is_set():
            # 서버로부터 데이터 요청
            # 현재 관절 각도 값: AXIS_ACT_MEAS
            response = kuka_client.read("AXIS_ACT_MEAS", False)
            # 현재 TCP 좌표 값: POS_ACT
            joints_list = await parse_joint_data_corrected(response)
            print(f"Joints from server for {robot.Name()}: {joints_list}")
            #print("서버로부터 받은 joints: " + str(joints_list))
            # 24.03.20 기준 현재 KUKA로부터 받은 joints값을 그대로 RoboDK에 반영할 경우
            # 일부 축에 대해서 반대 방향으로 움직이는 문제 발생
            # 반대 방향으로 움직이는 축: A1, A4, A5
            # 원인 파악 결과 KUKA 로봇 컨트롤러에서 해당 축에 대하여 방향을 변경함
            a, b, c = -joints_list[0], -joints_list[3], -joints_list[5]
            joints_list[0], joints_list[3], joints_list[5] = a, b, c
            print("실제 움직임에 받게 변경한 joints: " + str(joints_list))
            robot.setJoints(joints_list[:6])
            await asyncio.sleep(0.2)  # 비동기 대기
    finally:
        kuka_client.close()  # 클라이언트 연결 종료


async def main():
    # RoboDK 초기화
    RDK = Robolink()
    # 로봇 연결: Meltio
    robot1 = RDK.Item('KUKA KR 70 R2100-Meltio')
    tool1 = robot1.Tool()
    turntable1 = RDK.Item('2DOF Turn-table')
    reference1 = RDK.Item('Baseline')

    # 속도 변수 설정
    linear_speed = 10
    angular_speed = 180
    joints_speed = 5
    joints_accel = 40

    # 인스턴스 생성 및 초기화
    stop_event = Event()

    # 초기 설정
    robot1.setPoseFrame(reference1)
    robot1.setPoseTool(tool1)
    robot1.setSpeedJoints(joints_speed)
    print("Set Meltio")
    # 웹소켓 서버 태스크 생성
    robot_update_task_Meltio = asyncio.create_task(update_robot_angles(robot1, stop_event))

    # 서버 및 로봇 제어 태스크를 기다림
    await asyncio.gather(
        robot_update_task_Meltio)


# 웹소켓 통신 모듈 인스턴스 생성 및 서버 시작
if __name__ == "__main__":
    asyncio.run(main())
