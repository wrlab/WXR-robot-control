import asyncio
import websockets
import numpy as np
import json
from robodk.robomath import *
import time
import threading


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

        # IK 계산용 thread 생성 [robot1]
        self.position1 = None  # 위치 데이터 초기화
        self.rotation1 = None  # 회전 데이터 초기화
        self.reachable1 = True  # IK솔루션 있을 경우 초기화
        self.reachable_event1 = asyncio.Event()
        self.ik_results1 = None

        # IK 계산용 thread 생성 [robot2]
        self.position2 = None  # 위치 데이터 초기화
        self.rotation2 = None  # 회전 데이터 초기화
        self.reachable2 = True  # IK솔루션 있을 경우 초기화
        self.reachable_event2 = asyncio.Event()
        self.ik_results2 = None

        # 이동 명령 버퍼
        self.speed = 10 # default
        self.init_pose1 = self.robot1.Pose()
        self.current_pose1 = self.init_pose1.copy()
        self.buffer = []

        print("webSocket Start!")

    def cal_local_pose(self, position, rotation):
        local_pose = KUKA_2_Pose(
            [1000 * position['x'], -1000 * position['z'], 1000 * position['y'],
             rotation['x'], rotation['y'], rotation['z']])
        return local_pose

    def start_server(self):
        server = websockets.serve(self.handler, self.host, self.port)
        asyncio.get_event_loop().run_until_complete(server)
        asyncio.get_event_loop().run_forever()

    async def handler(self, websocket):
        receiver_task = asyncio.create_task(self.receive_messages(websocket))
        sender_task = asyncio.create_task(self.send_joint_positions(websocket))
        move_task = asyncio.create_task(self.move_tcp())

        await asyncio.gather(receiver_task, sender_task, move_task)

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

            if data.get("command") == "move":
                move_data = data.get("move")
                if move_data:
                    if move_data.get('x') is not None:
                        self.buffer.append(f"move_x_{move_data['x']}")
                    if move_data.get('y') is not None:
                        self.buffer.append(f"move_y_{move_data['y']}")
                    if move_data.get('z') is not None:
                        self.buffer.append(f"move_z_{move_data['z']}")


                    # if move_data.get('x') == '-':
                    #     self.buffer.append('move_x_minus')
                    # elif move_data.get('x') == '+':
                    #     self.buffer.append('move_x_plus')
                    # if move_data.get('y') == '-':
                    #     self.buffer.append('move_y_minus')
                    # elif move_data.get('y') == '+':
                    #     self.buffer.append('move_y_plus')
                    # if move_data.get('z') == '-':
                    #     self.buffer.append('move_z_minus')
                    # elif move_data.get('z') == '+':
                    #     self.buffer.append('move_z_plus')

            # if data.get("move") == "move_x_minus":
            #     self.buffer.append('move_x_minus')
            # elif data.get("move") == "move_x_plus":
            #     self.buffer.append('move_x_plus')
            # elif data.get("move") == "move_y_minus":
            #     self.buffer.append('move_y_minus')
            # elif data.get("move") == "move_y_plus":
            #     self.buffer.append('move_y_plus')
            # elif data.get("move") == "move_z_minus":
            #     self.buffer.append('move_z_minus')
            # elif data.get("move") == "move_z_plus":
            #     self.buffer.append('move_z_plus')

            if data.get("move") == "stop":
                print("Received stop command")
                self.buffer = []

    async def move_tcp(self):
        while True:
            if self.buffer:
                command = self.buffer.pop(0)
                print(f"Executing command: {command}")
                axis, direction = command.split('_')[1], command.split('_')[2]
                delta = self.speed if direction == '+' else -self.speed

                if axis == 'x':
                    self.current_pose1 = self.current_pose1 * transl(delta, 0, 0)
                elif axis == 'y':
                    self.current_pose1 = self.current_pose1 * transl(0, delta, 0)
                elif axis == 'z':
                    self.current_pose1 = self.current_pose1 * transl(0, 0, delta)


                # if command == 'move_x_minus':
                #     self.current_pose1 = self.current_pose1 * transl(-self.speed, 0, 0)
                # elif command == 'move_x_plus':
                #     self.current_pose1 = self.current_pose1 * transl(self.speed, 0, 0)
                # elif command == 'move_y_minus':
                #     self.current_pose1 = self.current_pose1 * transl(0, -self.speed, 0)
                # elif command == 'move_y_plus':
                #     self.current_pose1 = self.current_pose1 * transl(0, self.speed, 0)
                # elif command == 'move_z_minus':
                #     self.current_pose1 = self.current_pose1 * transl(0, 0, -self.speed)
                # elif command == 'move_z_plus':
                #     self.current_pose1 = self.current_pose1 * transl(0, 0, self.speed)

                # 로봇에 새 위치 데이터를 적용
                self.robot1.MoveL(self.current_pose1)

            await asyncio.sleep(0.001)  # 주기적으로 버퍼 확인

    async def send_joint_positions(self, websocket):
        print("send_joint_positions")
        while True:
            # await  self.reachable_event1.wait()  # reachable이 True가 될 때까지 대기

            # 관절 위치 전송 로직
            current_joints1 = self.robot1.Joints()
            current_tables1 = self.turntable1.Joints()

            current_joints2 = self.robot2.Joints()
            current_tables2 = self.turntable2.Joints()

            reachable1 = self.reachable1

            if not reachable1:
                json_data3 = json.dumps(reachable1)
                print(json_data3)
                await websocket.send(json_data3)
            else:
                # 로봇 관절 각도 값들이나 턴테이블의 각도값이 변하였을 때 데이터 전송
                if not np.array_equal(current_joints1, self.previous_joints1):
                    # 웹소켓 전송 시간 시작
                    # print("send_joint_positions of robot[1]")
                    start_time3 = time.time()
                    joints_np = np.array(current_joints1)
                    joints_flat = joints_np.flatten()

                    data = joints_flat.tolist()
                    data.append('robot1')
                    data.append(reachable1)
                    json_data = json.dumps(data)
                    await websocket.send(json_data)

                    self.previous_joints1 = current_joints1
            # else:
            #    print("Same as Joint1")

            if not np.array_equal(current_tables1, self.previous_tables1) and not self.on_table1:
                print("send_turntable_rotations of table[1]")
                tables_np = np.array(current_tables1)
                tables_flat = tables_np.flatten()
                data2 = tables_flat.tolist()
                json_data2 = json.dumps(data2)
                await websocket.send(json_data2)
                self.previous_tables1 = current_tables1

            if not np.array_equal(current_joints2, self.previous_joints2):
                # 웹소켓 전송 시간 시작
                # print("send_joint_positions of robot[2]")
                start_time3 = time.time()
                joints_np = np.array(current_joints2)
                joints_flat = joints_np.flatten()
                # print("joints_flat_r2: " + str(joints_flat))

                data = joints_flat.tolist()
                data.append('robot2')
                # print("data_r2: " + str(data))
                json_data = json.dumps(data)
                await websocket.send(json_data)

                self.previous_joints2 = current_joints2

            # send_joint_positions에서 webSocket을 독점하는 것을 막기 위해
            await asyncio.sleep(0.00001)
