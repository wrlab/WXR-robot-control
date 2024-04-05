import asyncio
import websockets
import numpy as np
import json
from robodk.robomath import *
import time
import threading

#socket_lock = threading.Lock()
#socket_lock2 = threading.Lock()


def rad2degree(rad):
    return rad * 180 / math.pi


class WebSocketCommunication:
    def __init__(self, host, port, robots, tools, ext_tools, rdk):
        self.host = host
        self.port = port
        self.robot1 = robots[0]
        self.tool1 = tools[0]
        self.turntable1 = ext_tools[0]
        self.robot2 = robots[1]
        self.tool2 = tools[1]
        self.turntable2 = ext_tools[1]

        print("tool1: " + str(self.tool1))
        print("tool2: " + str(self.tool2))

        # 2번째 로봇에 대한 부분은 아직 구현x 24.03.08기준
        self.previous_joints1 = None
        self.previous_tables1 = None
        self.on_table1 = False

        self.previous_joints2 = None
        self.previous_tables2 = None
        self.on_table2 = False

        self.rdk = rdk

        # IK 계산용 thread 생성 [robot2]
        self.position1 = None  # 위치 데이터 초기화
        self.rotation1 = None  # 회전 데이터 초기화
        self.reachable1 = True  # IK솔루션 있을 경우 초기화
        self.reachable_event1 = asyncio.Event()
        self.ik_results1 = None
        self.ik_thread1 = threading.Thread(
            target=self.ik_cal_thread,
            args=(self.robot1, self.tool1, self.position1, self.rotation1,
                  self.reachable_event1, self.ik_results1))
        self.ik_thread1.start()

        # IK 계산용 thread 생성 [robot2]
        self.position2 = None  # 위치 데이터 초기화
        self.rotation2 = None  # 회전 데이터 초기화
        self.reachable2 = True  # IK솔루션 있을 경우 초기화
        self.reachable_event2 = asyncio.Event()
        self.ik_results2 = None
        self.ik_thread2 = threading.Thread(
            target=self.ik_cal_thread,
            args=(self.robot2, self.tool2, self.position2, self.rotation2,
                  self.reachable_event2, self.ik_results2))
        self.ik_thread2.start()

        print("webSocket Start!")

    def ik_cal_thread(self, robot, tool, position, rotation, reachable_event, ik_result):
        while True:
            if position and rotation:
                local_pose = KUKA_2_Pose(
                    [1000 * position['x'], -1000 * position['z'], 1000 * position['y'],
                     rotation['x'], rotation['y'], rotation['z']])
                all_solutions = robot.SolveIK_All(local_pose, tool)

                # 2024.01.12 기준으로 if len(all_solutions) == 0: 이 부분으로 진입하지 않음 수정해야함!!
                if len(all_solutions) == 0:
                    print("IK unreachable!!")
                    # 현재는 메시지 출력으로 디버깅하고, 나중에는 단순 변수 변환으로 변경예정(2024.01.12)
                    # self.rdk.ShowMessage('There is no solution for requested position', True)
                    reachable = False
                    reachable_event.clear()
                    # if robot == self.robot1:
                    #     self.reachable1 = False
                    #     self.reachable_event1.clear()  # IK가 불가능할 때 이벤트 해제
                    # elif robot == self.robot2:
                    #     self.reachable2 = False
                    #     self.reachable_event2.clear()  # IK가 불가능할 때 이벤트 해제
                else:
                    for j in all_solutions:
                        conf_RLF = robot.JointsConfig(j).list()
                        rear, lower, flip = conf_RLF[:3]

                        if rear == 0 and lower == 0 and flip == 0:
                            #ik_result = j
                            #socket_lock.acquire()
                            robot.setJoints(j[:6])
                            #socket_lock.release()
                            # IK 계산이 완료된 후, position과 rotation을 None으로 설정
                            position = None
                            rotation = None
                            ik_result = j
                            #reachable = True
                            reachable_event.set()  # IK가 가능할 때 이벤트 설정
                            print("IK reachable!!")
                            break
            time.sleep(0.00001)

    # def ik_cal_thread2(self):
    #     while True:
    #         if self.position2 and self.rotation2:
    #             # print("ik_cal Working!!")
    #             local_pose = KUKA_2_Pose(
    #                 [1000 * self.position2['x'], -1000 * self.position2['z'], 1000 * self.position2['y'],
    #                  self.rotation2['x'], self.rotation2['y'], self.rotation2['z']])
    #             all_solutions = self.robot2.SolveIK_All(local_pose, self.tool2)
    #
    #             # 2024.01.12 기준으로 if len(all_solutions) == 0: 이 부분으로 진입하지 않음 수정해야함!!
    #             if len(all_solutions) == 0:
    #                 print("IK unreachable!!")
    #                 # 현재는 메시지 출력으로 디버깅하고, 나중에는 단순 변수 변환으로 변경예정(2024.01.12)
    #                 # self.rdk.ShowMessage('There is no solution for requested position', True)
    #                 self.reachable2 = False
    #                 self.reachable_event2.clear()  # IK가 불가능할 때 이벤트 해제
    #             else:
    #                 for j in all_solutions:
    #                     conf_RLF = self.robot2.JointsConfig(j).list()
    #
    #                     rear = conf_RLF[0]  # 1 if Rear , 0 if Front
    #                     lower = conf_RLF[1]  # 1 if Lower, 0 if Upper (elbow)
    #                     flip = conf_RLF[2]  # 1 if Flip , 0 if Non flip (Flip is usually when Joint 5 is negative)
    #
    #                     if rear == 0 and lower == 0 and flip == 0:
    #                         self.ik_results2 = j
    #                         #socket_lock2.acquire()
    #                         self.robot2.setJoints(self.ik_results2[:6])
    #                         #socket_lock2.release()
    #                         # IK 계산이 완료된 후, position과 rotation을 None으로 설정
    #                         self.position2 = None
    #                         self.rotation2 = None
    #                         self.reachable2 = True
    #                         self.reachable_event2.set()  # IK가 가능할 때 이벤트 설정
    #                         print("IK reachable!!")
    #                         break
    #
    #         time.sleep(0.00001)

    def start_server(self):
        server = websockets.serve(self.handler, self.host, self.port)
        asyncio.get_event_loop().run_until_complete(server)
        asyncio.get_event_loop().run_forever()

    async def handler(self, websocket):
        receiver_task = asyncio.create_task(self.receive_messages(websocket))
        sender_task = asyncio.create_task(self.send_joint_positions(websocket))

        await asyncio.gather(receiver_task, sender_task)

    async def receive_messages(self, websocket):
        async for message in websocket:
            data = json.loads(message)

            if data.get("command") == "start_streaming":
                print("Start streaming command received")

            if data.get("command") == "onoff_turntable":
                # self.on_table True 또는 False 받기
                self.on_table1 = data.get("onoff")

            if data.get("command") == "update_turntables":
                joint1 = data.get("joint1")
                x1 = joint1.get("x")
                y1 = joint1.get("y")
                z1 = joint1.get("z")
                joint2 = data.get("joint2")
                y2 = joint2.get("y")
                # radian -> degree
                x1_degree = rad2degree(x1)
                y2_degree = rad2degree(y2)

                if (y1 == -180 and z1 == -180) or (y1 == 180 and z1 == 180):
                    joint_values = [x1_degree, y2_degree]
                    self.turntable1.setJoints(joint_values)
                elif (y1 == -180 and z1 == 180) or (y1 == 180 and z1 == -180):
                    joint_values = [-x1_degree, y2_degree]
                    self.turntable1.setJoints(joint_values)
                else:
                    joint_values = [-x1_degree, y2_degree]
                    self.turntable1.setJoints(joint_values)
                print(f"update_turntables")

            if data.get("command") == "update_position":
                print("Get update_position")
                # 2024.01.02
                self.position1 = data.get("position")
                self.rotation1 = data.get("rotation")

            if data.get("command") == "update_position2":
                print("Get update_position2")
                # 2024.03.08
                self.position2 = data.get("position")
                self.rotation2 = data.get("rotation")
                print("TCP2 Position: " + str(self.position2))
                print("TCP2 Rotation: " + str(self.rotation2))

    async def send_joint_positions(self, websocket):
        print("send_joint_positions")
        while True:
            #await  self.reachable_event1.wait()  # reachable이 True가 될 때까지 대기

            # 관절 위치 전송 로직
            #socket_lock.acquire()
            current_joints1 = self.robot1.Joints()
            print(current_joints1)
            #current_tables1 = self.turntable1.Joints()
            #socket_lock.release()

            #socket_lock2.acquire()
            current_joints2 = self.robot2.Joints()
            #current_tables2 = self.turntable2.Joints()
            #socket_lock2.release()

            reachable1 = self.reachable1


            # 로봇 관절 각도 값들이나 턴테이블의 각도값이 변하였을 때 데이터 전송
            if not np.array_equal(current_joints1, self.previous_joints1):
                # 웹소켓 전송 시간 시작
                print("send_joint_positions of robot[1]")
                start_time3 = time.time()
                joints_np = np.array(current_joints1)
                joints_flat = joints_np.flatten()

                data = joints_flat.tolist()
                data.append('robot1')
                json_data = json.dumps(data)
                await websocket.send(json_data)

                self.previous_joints1 = current_joints1
            else:
                print("Same as Joint1")

            # if not np.array_equal(current_tables1, self.previous_tables1) and not self.on_table1:
            #     print("send_turntable_rotations of table[1]")
            #     tables_np = np.array(current_tables1)
            #     tables_flat = tables_np.flatten()
            #     data2 = tables_flat.tolist()
            #     json_data2 = json.dumps(data2)
            #     await websocket.send(json_data2)
            #     self.previous_tables1 = current_tables1

            if not np.array_equal(current_joints2, self.previous_joints2):
                # 웹소켓 전송 시간 시작
                print("send_joint_positions of robot[2]")
                start_time3 = time.time()
                joints_np = np.array(current_joints2)
                joints_flat = joints_np.flatten()
                print("joints_flat_r2: " + str(joints_flat))

                data = joints_flat.tolist()
                data.append('robot2')
                print("data_r2: " + str(data))
                json_data = json.dumps(data)
                await websocket.send(json_data)

                self.previous_joints2 = current_joints2

            # if not np.array_equal(current_tables2, self.previous_tables2) and not self.on_table2:
            #     print("send_turntable_rotations of table[2]")
            #     tables_np = np.array(current_tables2)
            #     tables_flat = tables_np.flatten()
            #     data2 = tables_flat.tolist()
            #     json_data2 = json.dumps(data2)
            #     await websocket.send(json_data2)
            #     self.previous_tables2 = current_tables2

            json_data3 = json.dumps(reachable1)
            await websocket.send(json_data3)

            # send_joint_positions에서 webSocket을 독점하는 것을 막기 위해
            await asyncio.sleep(0.00001)
