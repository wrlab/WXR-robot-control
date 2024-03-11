from robodk.robolink import *
import asyncio
from asyncio import Event


async def parsing_data(response):
    # 파싱할 데이터가 저장될 데이터
    parsed_data = None

    return parsed_data

async def rdk_update_robot_angles(robot, stop_event):
    # 로봇과의 연결 확인
    if not robot.Connected():
        # 로봇과 연결 시도
        success = robot.Connect()
        if success:
            print("로봇에 성공적으로 연결되었습니다.")
        else:
            print("로봇과의 연결에 실패하였습니다. 설정을 확인해주세요.")
    else:
        print("이미 로봇과 연결되어 있습니다.")

    while not stop_event.is_set():
        # 서버로부터 데이터 요청
        joints_str = robot.setParam("Driver", "GET $AXIS_ACT_MEAS")
        print(f"서버로부터 받은 관절각도: {joints_str}")
        if robot:
            robot.setJoints(joints_str)
        await asyncio.sleep(0.5)  # 비동기 대기
    return

async def main():
    # 로봇 연결
    RDK = Robolink()
    robot = RDK.Item('KUKA KR 70 R2100-Meltio')
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
    robot_update_task = asyncio.create_task(rdk_update_robot_angles(robot, stop_event))

    # 서버 및 로봇 제어 태스크를 기다림
    await asyncio.gather(robot_update_task)

# 웹소켓 통신 모듈 인스턴스 생성 및 서버 시작
if __name__ == "__main__":
    asyncio.run(main())