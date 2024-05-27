from robodk.robolink import *
from robodk.robomath import *
import time
import asyncio

linear_speed = 10
angular_speed = 180
joints_speed = 5
joints_accel = 40
is_stop = False


class Rdk2KukaControl:
    def __init__(self):
        self.RDK = Robolink()
        self.robot = self.RDK.Item('KUKA KR 70 R2100-Meltio')
        self.tool = self.robot.Tool()
        self.reference = self.RDK.Item('Baseline')
        self.tcp_obj = self.RDK.Item('tcp_obj')
        self.bbox = self.RDK.Item('bbox')
        self.current_joints = self.robot.SimulatorJoints()

        # bool
        self.is_collide = False
        self.within_bbox = True
        self.is_order2kuka = False

        # Boundary limits
        self.min_x, self.max_x, self.min_y, self.max_y, self.min_z, self.max_z = [-90, 90, -90, 90, 210, 390]

        # Initial robot settings
        self.robot.setPoseFrame(self.reference)
        self.robot.setPoseTool(self.tool)
        self.robot.setSpeedJoints(joints_speed)

    async def order2kuka(self):
        while not is_stop:
            start_time = time.time()
            joints = self.robot.SimulatorJoints()
            print("current joints: ", self.current_joints)
            print("simulatorJoints: ", joints)
            if (self.current_joints != joints) and (not self.is_collide and self.within_bbox):
                if self.is_order2kuka:
                    # kuka_controller 에 이동 명령 전달
                    self.RDK.setRunMode(RUNMODE_RUN_ROBOT)
                    print("Change joint values are: ", joints)
                    self.robot.MoveJ(joints)
                    print("Robot MoveJ")
                    self.current_joints = joints
                    self.RDK.setRunMode(RUNMODE_SIMULATE)
                else:
                    print("Don't send data.")


                # Execution time
                print(f"Execution time: {time.time() - start_time} seconds")
            else:
                print("Not order2kuka!")

            await asyncio.sleep(0.01)

    async def run(self):
        user_input_bbox = input("test_bbox를 진행하시겠습니까? (yes/no): ")

        user_input_order2kuka = input("KUKA로봇에 이동 명령을 전달합니까? (yes/no): ")

        if user_input_bbox.lower() == "yes":
            print("test_bbox를 진행합니다.")
            await self.test_bbox()
        else:
            print("test_bbox를 취소합니다.")

        if user_input_order2kuka.lower() == "yes":
            print("KUKA로봇에게 이동 명령을 전달합니다.")
            self.is_order2kuka = True
        else:
            print("KUKA로봇에게 이동 명령을 전달하지 않습니다..")
            self.is_order2kuka = False

        # 각 기능을 독립적인 태스크로 실행
        order_task = asyncio.create_task(self.order2kuka())

        self.RDK.setRunMode(RUNMODE_RUN_ROBOT)
        print("Runmode 설정")
        self.RDK.Render(False)

        while not is_stop:
            # 조건을 체크하여 프로그램 종료 여부 결정
            if not self.is_collide and self.within_bbox:
                print("조건 충족, 작업 계속 진행")
            else:
                print("경계를 벗어나거나 충돌 발생, 프로그램 종료")
                # 태스크를 취소
                order_task.cancel()
                break  # while 루프 종료

            # 짧은 대기 시간을 주어 CPU 사용률을 관리
            await asyncio.sleep(0.05)

        # 모든 태스크가 완료될 때까지 기다림
        await asyncio.gather(order_task, return_exceptions=True)

        print('Rdk2KukaControl Program has ended.')


if __name__ == "__main__":
    kuka_controller = Rdk2KukaControl()

    # 비동기 이벤트 루프 시작
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(kuka_controller.run())
    finally:
        loop.close()