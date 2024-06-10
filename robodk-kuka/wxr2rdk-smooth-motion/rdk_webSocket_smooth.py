import asyncio
import websockets
import numpy as np
import json
from robodk.robomath import *
from robodk.robolink import *
import time
import threading

socket_lock = threading.Lock()
socket_lock2 = threading.Lock()


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

        self.current_joints = self.robot1.SimulatorJoints()

        # smooth parameters
        self.Ts = 0.01 # sampling 주기
        self.kp3 = 30 # 3단계 위치 보정
        self.kd3 = 4.2 # 3단계 속도 보정
        #self.u3_plus = 0.01 # 3단계 슬라이딩 모드 스위칭
        self.u3_plus = 0.0  # 3단계 슬라이딩 모드 스위칭
        self.home = [0, -90, 90, 0, 0, 0]
        #self.target_joints_rad = [radians(joint) for joint in self.home]
        #print(self.target_joints_rad)

        #self.start_ik_threads()

        print("webSocket Start!")

    #def start_ik_threads(self):
        #threading.Thread(target=self.tool_teleoperation, args=(1,)).start()
        #threading.Thread(target=self.tool_teleoperation, args=(2,)).start()

    #def tool_teleoperation(self, robot_id):
    async def tool_teleoperation(self, robot_id):
        while True:
            start_time = time.time()
            robot = getattr(self, f'robot{robot_id}')
            tool = getattr(self, f'tool{robot_id}')
            position = getattr(self, f'position{robot_id}')
            rotation = getattr(self, f'rotation{robot_id}')

            if position and rotation:
                current_pose = robot.Pose()
                print("rotation: ", rotation)
                new_pose = self.cal_new_pose(current_pose, position, rotation, robot, robot_id)
                print("new_pose: ", new_pose)

                # try:
                #     robot.MoveJ(new_pose)
                #     break
                # except StoppedError as e:
                #     print(f"Collision detected for robot {robot_id}, waiting to retry")
                #     robot.MoveJ(self.home)
                    #robot.MoveJ(self.home)
                    # while True:
                    #     try:
                    #         time.sleep(1)
                    #         #robot.MoveJ(self.home)
                    #         break
                    #     except StoppedError:
                    #         print("Still in collision, waiting...")
                    #         continue

                #self.rdk.Render(False)
                #self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
                robot.MoveJ(new_pose)
                self.rdk.setRunMode(RUNMODE_SIMULATE)


                # all_solutions = robot.SolveIK_All(new_pose, tool)
                # if len(all_solutions) == 0:
                #     print("There are no all_solutions ")
                # else:
                #     print("all_solutions: ", len(all_solutions))
                #     for j in all_solutions:
                #         conf_rlf = robot.JointsConfig(j).list()
                #         rear, lower, flip = conf_rlf[:3]
                #         if rear == 0 and lower == 0 and flip == 1:
                #             print("Front, elbow up, flip solution")
                #             try:
                #                 robot.MoveJ(j)
                #                 break
                #             except StoppedError as e:
                #                 print(f"Collision detected for robot {robot_id}, waiting to retry")
                #                 robot.MoveJ(self.home)
                #                 while True:
                #                     try:
                #                         time.sleep(1)
                #                         robot.MoveJ(self.home)
                #                         break
                #                     except StoppedError:
                #                         print("Still in collision, waiting...")
                #                         continue

            #time.sleep(self.Ts)
            #time.sleep(0.01)
            await asyncio.sleep(0.01)

    def cal_local_pose(self, position, rotation):
        local_pose = KUKA_2_Pose(
            [1000 * position['x'], -1000 * position['z'], 1000 * position['y'],
             rotation['x'], rotation['y'], rotation['z']])
        return local_pose

    def get_rotation_mat(self, pose):
        # # Pose 객체에서 회전 행렬을 추출
        # rotation_matrix = pose[:3, :3]
        # Pose 객체에서 회전 행렬을 추출
        rotation_matrix = np.array(pose).reshape(4, 4)[:3, :3]
        return rotation_matrix

    def cal_new_pose(self, current_pose, position, rotation, robot, robot_id):
        # 목표 위치와 회전을 로컬 포즈로 변환
        local_pose = self.cal_local_pose(position, rotation)

        #tool = getattr(self, f'tool{robot_id}')

        # 목표 위치와 현재 위치의 차이 계산
        target_position = [local_pose.Pos()[0], local_pose.Pos()[1], local_pose.Pos()[2]]
        error = np.array(target_position) - np.array(current_pose.Pos())

        # 슬라이딩 모드 제어 신호 계산
        norm_error = np.linalg.norm(error)
        if norm_error != 0:
            sliding_term = self.u3_plus * error / norm_error
        else:
            sliding_term = self.u3_plus * error

        # 관절 속도 추정
        #joint_velocities = self.get_robot_vel(robot_id)

        # 위치 보정 신호 계산
        velocity_command = self.kp3 * error + sliding_term

        # 목표 자세 계산
        new_position = np.array(current_pose.Pos()) + velocity_command * self.Ts
        print("new_position: ", new_position)

        x_rotation = rotation.get('x')
        y_rotation = rotation.get('y')
        z_rotation = rotation.get('z')

        # # Initialize rotation matrices
        # x_rot_matrix = rotx(0)
        # y_rot_matrix = roty(0)
        # z_rot_matrix = rotz(0)

        x_rot_matrix = rotx(x_rotation)  # Use radians for rotation
        z_rot_matrix = rotz(y_rotation)  # Use radians for rotation
        y_rot_matrix = roty(-z_rotation)  # Use radians for rotation
        rotation_matrix = x_rot_matrix * y_rot_matrix * z_rot_matrix

        new_pose = current_pose
        new_pose.setPos(new_position)

        # Apply the rotation to the current pose
        new_pose = transl(new_pose.Pos()) * rotation_matrix
        print("new_pose: ", new_pose)

        return new_pose

    def estimate_joint_vel(self, previous_joints, current_joints, Ts):
        if previous_joints is None:
            return np.zeros_like(current_joints)
        return (np.array(current_joints)- np.array(previous_joints)) / Ts

    def get_robot_vel(self, robot_id):
        current_joints = getattr(self, f'robot{robot_id}').Joints()
        previous_joints = getattr(self, f'previous_joints{robot_id}')

        # 관절 속도 추정
        joint_velocities = self.estimate_joint_vel(previous_joints, current_joints, self.Ts)

        # 이전 위치 업데이트
        setattr(self, f'previous_joints{robot_id}', current_joints)

        return joint_velocities

    def set_rotation(self, rotate_date):
        x_rot = rotate_date.get('x')
        y_rot = rotate_date.get('y')
        z_rot = rotate_date.get('z')

        # Initialize rotation matrices
        x_rot_matrix = rotx(0)
        y_rot_matrix = roty(0)
        z_rot_matrix = rotz(0)



    def start_server(self):
        server = websockets.serve(self.handler, self.host, self.port)
        asyncio.get_event_loop().run_until_complete(server)
        asyncio.get_event_loop().run_forever()

    async def handler(self, websocket):
        receiver_task = asyncio.create_task(self.receive_messages(websocket))
        sender_task = asyncio.create_task(self.send_joint_positions(websocket))
        #order_task = asyncio.create_task(self.order2kuka())
        operation_task = asyncio.create_task(self.tool_teleoperation(1))

        await asyncio.gather(receiver_task, sender_task,
                             operation_task
                             #,order_task
                             )

    async def receive_messages(self, websocket):
        async for message in websocket:
            data = json.loads(message)
            #self.robot1.Stop()
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
                #print(f"update_turntables")

            if data.get("command") == "update_position":
                #print("Get update_position")
                # 2024.01.02
                self.position1 = data.get("position")
                self.rotation1 = data.get("rotation")
                #print("TCP1 Position: " + str(self.position1))
                print("TCP1 Rotation: " + str(self.rotation1))

            if data.get("command") == "update_position2":
                #print("Get update_position2")
                # 2024.03.08
                self.position2 = data.get("position")
                self.rotation2 = data.get("rotation")
                #print("TCP2 Position: " + str(self.position2))
                #print("TCP2 Rotation: " + str(self.rotation2))

            #await asyncio.sleep(0.0001)

    async def send_joint_positions(self, websocket):
        print("send_joint_positions")
        while True:
            # await  self.reachable_event1.wait()  # reachable이 True가 될 때까지 대기

            # 관절 위치 전송 로직
            #socket_lock.acquire()
            current_joints1 = self.robot1.Joints()
            # print(current_joints1)
            current_tables1 = self.turntable1.Joints()
            #socket_lock.release()

            # socket_lock2.acquire()
            current_joints2 = self.robot2.Joints()
            current_tables2 = self.turntable2.Joints()
            # socket_lock2.release()

            reachable1 = self.reachable1

            if not reachable1:
                json_data3 = json.dumps(reachable1)
                #data_np = np.array(json_data3)
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

            # turn-table 값 전달
            # if not np.array_equal(current_tables2, self.previous_tables2) and not self.on_table2:
            #     print("send_turntable_rotations of table[2]")
            #     tables_np = np.array(current_tables2)
            #     tables_flat = tables_np.flatten()
            #     data2 = tables_flat.tolist()
            #     json_data2 = json.dumps(data2)
            #     await websocket.send(json_data2)
            #     self.previous_tables2 = current_tables2

            # json_data3 = json.dumps(reachable1)
            # print(json_data3)
            # await websocket.send(json_data3)

            # send_joint_positions에서 webSocket을 독점하는 것을 막기 위해
            await asyncio.sleep(0.001)

    async def order2kuka(self):
        while True:
            start_time = time.time()
            joints = self.robot1.SimulatorJoints()
            print("current joints: ", self.current_joints)
            print("simulatorJoints: ", joints)
            #if self.current_joints != joints:
            if not np.allclose(self.current_joints, joints, atol=0.1):
                socket_lock.acquire()
                # kuka_controller 에 이동 명령 전달
                print("RunMode: ", self.rdk.RunMode())
                self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
                print("changed RunMode: ", self.rdk.RunMode())
                print("Change joint values are: ", joints)
                self.robot1.MoveJ(joints)
                print("Robot MoveJ")
                self.current_joints = joints
                self.rdk.setRunMode(RUNMODE_SIMULATE)
                socket_lock.release()

                # Execution time
                print(f"Execution time: {time.time() - start_time} seconds")
            else:
                print("Not order2kuka!")

            await asyncio.sleep(1)
