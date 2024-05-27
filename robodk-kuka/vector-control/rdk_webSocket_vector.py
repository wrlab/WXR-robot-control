import asyncio
import math

import websockets
import numpy as np
import json
from robodk.robolink import *
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
        # self.rdk.Render(False)
        # self.rdk.setRunMode(RUNMODE_RUN_ROBOT)

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
        self.allow_range = 0.1745

        #self.start_ik_threads()

        print("webSocket Start!")

    # def start_ik_threads(self):
    #     threading.Thread(target=self.ik_cal_thread, args=(1,)).start()
    #
    # def ik_cal_thread(self, robot_id):
    #     while True:
    #         robot = getattr(self, f'robot{robot_id}')
    #         tool = getattr(self, f'tool{robot_id}')
    #         position = getattr(self, f'position{robot_id}')
    #         rotation = getattr(self, f'rotation{robot_id}')
    #
    #         if position and rotation:
    #             # IK 계산 로직
    #             local_pose = self.cal_local_pose(position, rotation)
    #
    #             all_solutions = robot.SolveIK_All(local_pose, tool)
    #
    #             if len(all_solutions) == 0:
    #                 print("IK unreachable!!")
    #                 self.reachable1 = False
    #             else:
    #                 for j in all_solutions:
    #                     conf_rlf = robot.JointsConfig(j).list()
    #                     rear, lower, flip = conf_rlf[:3]
    #
    #                     if rear == 0 and lower == 0 and flip == 1:
    #                         robot.MoveJ(j[:6])
    #                         #robot.setJoints(j[:6])
    #                         self.reachable1 = True
    #                         print("IK reachable!!")
    #                         # IK 결과를 인스턴스 변수에 저장
    #                         setattr(self, f'ik_results{robot_id}', j[:6])
    #                         break
    #                     else:
    #                         print("IK unreachable!!")
    #                         self.reachable1 = False
    #
    #             # 다음 IK 계산을 위해 위치와 회전 데이터 초기화
    #             setattr(self, f'position{robot_id}', None)
    #             setattr(self, f'rotation{robot_id}', None)
    #
    #         time.sleep(0.001)  # 스레드가 너무 빠르게 반복하지 않도록 적당한 지연
    def set_rotation(self, rotate_data):
        x_rotation = rotate_data.get('x')
        y_rotation = rotate_data.get('y')
        z_rotation = rotate_data.get('z')

        # Initialize rotation matrices
        x_rot_matrix = rotx(0)
        y_rot_matrix = roty(0)
        z_rot_matrix = rotz(0)

        # Update rotation matrices with provided angles
        if x_rotation is not None:
            print("x rotation: ", x_rotation)
            x_rot_matrix = rotx(x_rotation)  # Use radians for rotation
        if y_rotation is not None:
            print("y rotation: ", y_rotation)
            z_rot_matrix = rotz(y_rotation)  # Use radians for rotation
        if z_rotation is not None:
            print("z rotation: ", z_rotation)
            y_rot_matrix = roty(-z_rotation)  # Use radians for rotation

        # Combine the rotations
        rotation_matrix = x_rot_matrix * y_rot_matrix * z_rot_matrix

        # Apply the rotation to the current pose
        rotate_pose = transl(self.current_pose1.Pos()) * rotation_matrix

        # Check if all rotations are within the allowed range
        if (x_rotation is not None and -self.allow_range <= x_rotation <= self.allow_range and
                y_rotation is not None and -self.allow_range <= y_rotation <= self.allow_range and
                z_rotation is not None and -self.allow_range <= z_rotation <= self.allow_range):
            self.rdk.Render(False)
            self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
            self.robot1.MoveJ(rotate_pose)
            self.rdk.setRunMode(RUNMODE_SIMULATE)
        else:
            print("One or more rotation values are out of the allowed range. MoveJ command not executed.")
            print()


        # self.rdk.Render(False)
        # self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
        # self.robot1.MoveJ(rotate_pose)
        # self.rdk.setRunMode(RUNMODE_SIMULATE)


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
                #print("Get update_position")
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
                rotate_data = data.get("rotate")
                move_command = 'move'
                if move_data:
                    for axis in ['x', 'y', 'z']:
                        if move_data.get(axis) is not None:
                            move_command += f"_{axis}{move_data[axis]}"
                    self.buffer.append(move_command)

            if data.get("command") == "rotation":
                rotate_data = data.get("rotate")
                if rotate_data:
                    print("rotate data: ", rotate_data)
                    #self.set_rotation(rotate_data)

            if data.get("move") == "stop":
                print("Received stop command")
                self.buffer = []

    async def move_tcp(self):
        while True:
            if self.buffer:
                command = self.buffer.pop(0)
                print(f"Executing command: {command}")
                if command.startswith('move'):
                    movements = command.split('_')[1:]  # ['x+', 'y+', 'z-']
                    deltas = {'x': 0, 'y': 0, 'z': 0}
                    for movement in movements:
                        axis = movement[0]
                        direction = movement[1]
                        delta = self.speed if direction == '+' else -self.speed
                        deltas[axis] += delta
                    # 이동 명령 실행
                    self.current_pose1 = self.current_pose1 * transl(deltas['x'], deltas['y'], deltas['z'])
                    print("Updated pose: ", self.current_pose1)

                # elif command.startswith('rotate'):
                #     rotations = command.split('_')[1:]  # ['x10', 'y-5', 'z20']
                #     rotation_matrix = self.current_pose1
                #     euler_rotations = {'x': 0, 'y': 0, 'z': 0}
                #
                #     for rotation in rotations:
                #         axis = rotation[0]
                #         direction = rotation[1]
                #         euler = 1 if direction == '+' else -1
                #         euler_rotations[axis] += math.radians(euler)
                #         #rad_angle = math.radians(angle)  # 라디안으로 변환
                #         if axis == 'x':
                #             rotation_matrix = rotation_matrix * rotx(euler_rotations['x'])
                #         elif axis == 'y':
                #             rotation_matrix = rotation_matrix * roty(euler_rotations['y'])
                #         elif axis == 'z':
                #             rotation_matrix = rotation_matrix * rotz(euler_rotations['z'])
                #
                #     self.current_pose1 = rotation_matrix
                #     print("Updated rotation: ", self.current_pose1)
                self.rdk.Render(False)
                self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
                #self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
                #self.robot1.setPose(self.current_pose1)

                self.robot1.MoveJ(self.current_pose1)
                #self.robot1.MoveL(self.current_pose1)
                self.rdk.setRunMode(RUNMODE_SIMULATE)

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
            await asyncio.sleep(0.001)
