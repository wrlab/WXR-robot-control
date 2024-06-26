import asyncio
import websockets
import numpy as np
import json
from robodk.robomath import *
from robodk.robolink import *
import time
import threading
import ssl

socket_lock = threading.Lock()
socket_lock2 = threading.Lock()
pose_lock = asyncio.Lock()


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
        #self.kp3 = 2  # 3단계 위치 보정
        self.kd3 = 4.2 # 3단계 속도 보정
        self.u3_plus = 0.01 # 3단계 슬라이딩 모드 스위칭
        #self.u3_plus = 0.01  # 3단계 슬라이딩 모드 스위칭

        self.isTeleoper = True
        #self.program = self.rdk.Item("Prog1")
        self.program = self.rdk.Item("Prog2")
        self.inProgress = False
        self.home = self.rdk.Item("Home2")
        self.isCollision = False
        #self.home = [0, -90, 90, 0, 0, 0]

        # 24.06.11 - new variables
        self.current_pose = self.robot1.Pose()
        self.num = 0
        self.command_queue = asyncio.Queue()

        print("webSocket Start!")

    #def tool_teleoperation(self, robot_id):
    async def tool_teleoperation(self, robot_id):
        threshold = 1.0 # 임계값 설정 (단위: mm)
        #threshold = 0.0000001  # 임계값 설정 (단위: mm)
        #robot = getattr(self, f'robot{robot_id}')
        #current_pose = robot.Pose()

        while True:
            if (self.isTeleoper):
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

                    self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
                    robot.MoveJ(new_pose)
                    self.rdk.setRunMode(RUNMODE_SIMULATE)

                await asyncio.sleep(0.08)
            else:
                await asyncio.sleep(0.08)

                # if position and rotation:
                #     async with pose_lock: # Lock to ensure new_pose is executed sequentially
                #         current_pose_tmp = robot.Pose()
                #         new_pose = self.cal_new_pose(current_pose_tmp, position, rotation, robot, robot_id)
                #
                #         if self.pose_dif(self.current_pose, new_pose) > threshold:
                #             #socket_lock.acquire()
                #             self.num += 1
                #             print(f"MoveJ! {self.num} at {time.time()}")
                #             #self.rdk.Render(False)
                #             #self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
                #             #print("new_pose: ", new_pose)
                #
                #             ##############################
                #             self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
                #             robot.MoveJ(new_pose)
                #             self.rdk.setRunMode(RUNMODE_SIMULATE)
                #             self.current_pose = new_pose
                #             ##############################
                #
                #             #socket_lock.release()
                #             #self.current_pose = robot.Pose()
                #
                #             ##########
                #             #await self.command_queue.put(new_pose)
                #             ##########
                #
                #             #await self.execute_commands(robot)
                #
                # await asyncio.sleep(0.01)
            #     await self.execute_commands(robot)
            #
            # else:
            #     await asyncio.sleep(0.001)

    # async def execute_commands(self, robot):
    #     while not self.command_queue.empty():
    #         if (self.isTeleoper):
    #             new_pose = await self.command_queue.get()
    #             #self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
    #             async with pose_lock:
    #                 try:
    #                     self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
    #                     robot.MoveJ(new_pose)
    #                     self.rdk.setRunMode(RUNMODE_SIMULATE)
    #                 except StoppedError as e:
    #                     print(f"StoppedError: {e}")
    #                     if "Collision detected" in str(e):
    #                         print("collsion detected exception")
    #                         self.isTeleoper = False  # 충돌 감지 시 미러링 모드로 전환
    #                         self.isCollision = True
    #                         self.rdk.setCollisionActive(0)
    #                         self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
    #                         self.robot1.MoveJ(self.home)
    #                         self.rdk.setRunMode(RUNMODE_SIMULATE)
    #                         self.rdk.setCollisionActive(1)
    #                         self.isCollision = False
    #                 #self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
    #                 #robot.MoveJ(new_pose)
    #                 self.rdk.setRunMode(RUNMODE_SIMULATE)
    #                 self.current_pose = new_pose
    #                 await asyncio.sleep(0.01)
    #         else:
    #             await asyncio.sleep(0.01)
    #
    # async def wait_for_motion_complete(self, robot):
    #     while robot.Busy():
    #         await asyncio.sleep(0.0001)

    def pose_dif(self, pose1, pose2):
        p1 = Pose_2_KUKA(pose1)
        p2 = Pose_2_KUKA(pose2)
        robomath.distance(p1, p2)

        return robomath.distance(p1, p2)

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

        # 목표 위치와 현재 위치의 차이 계산
        #target_position = [local_pose.Pos()[0], local_pose.Pos()[1], local_pose.Pos()[2]]
        target_position = [local_pose.Pos()[0], -local_pose.Pos()[1], local_pose.Pos()[2]]
        error = np.array(target_position) - np.array(current_pose.Pos())

        # 슬라이딩 모드 제어 신호 계산
        norm_error = np.linalg.norm(error)
        if norm_error != 0:
            sliding_term = self.u3_plus * error / norm_error
        else:
            sliding_term = self.u3_plus * error

        # 위치 보정 신호 계산
        velocity_command = self.kp3 * error + sliding_term

        # 목표 자세 계산
        new_position = np.array(current_pose.Pos()) + velocity_command * self.Ts
        #print("new_position: ", new_position)

        x_rotation = rotation.get('x')
        y_rotation = rotation.get('y')
        z_rotation = rotation.get('z')

        x_rot_matrix = rotx(x_rotation)  # Use radians for rotation
        z_rot_matrix = rotz(y_rotation)  # Use radians for rotation
        y_rot_matrix = roty(-z_rotation)  # Use radians for rotation
        rotation_matrix = x_rot_matrix * y_rot_matrix * z_rot_matrix

        new_pose = current_pose
        new_pose.setPos(new_position)

        # Apply the rotation to the current pose
        new_pose = transl(new_pose.Pos()) * rotation_matrix
        #print("new_pose: ", new_pose)

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
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_context.load_cert_chain(certfile="C:/GitProjects/robodk-kuka/wxr2rdk-smooth-motion/selfsigned.crt", keyfile="C:/GitProjects/robodk-kuka/wxr2rdk-smooth-motion/selfsigned.key")
        server = websockets.serve(self.handler, self.host, self.port, ssl=ssl_context)
        #server = websockets.serve(self.handlerort)
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
            print("data: ", data)
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

            if data.get("command") == "setting mode":
                mode = data.get("mode")
                print("data: ", data)
                if mode == "Remote":
                    print("mode: ", mode)
                    self.isTeleoper = True
                elif mode == "Mirroring":
                    print("mode: ", mode)
                    self.isTeleoper = False

            if data.get("command") == 'Simulation':
                signal = data.get("signal")
                if signal == 'Start':
                    print("Start Simulation Program")
                    # self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
                    self.program.RunProgram()
                    self.rdk.setRunMode(RUNMODE_SIMULATE)
                    self.inProgress = True
                    sim_data = json.dumps(self.inProgress)
                    print("sim_data: ", sim_data)
                    await websocket.send(sim_data)
                    while self.program.Busy():
                        print("Program is running...")
                        await asyncio.sleep(0.5)
                        try:
                            message = await asyncio.wait_for(websocket.recv(), timeout=0.001)
                            data = json.loads(message)
                            if data.get("command") == 'Simulation' and data.get("signal") == 'Stop':
                                await self.stop_simulation(websocket)
                                break
                        except asyncio.TimeoutError:
                            continue
                    print("Program has Finished.")
                    self.inProgress = False
                    sim_data = json.dumps(self.inProgress)
                    print("sim_data: ", sim_data)
                    sim_finished = json.dumps("sim_finished")
                    await websocket.send(sim_data)
                    await websocket.send(sim_finished)
                elif signal == 'Stop':
                    await self.stop_simulation(websocket)

            if data.get("command") == "update_position":
                #print("Get update_position")
                # 2024.01.02
                self.position1 = data.get("position")
                self.rotation1 = data.get("rotation")

            #await asyncio.sleep(0.0001)

    async def stop_simulation(self, websocket):
        print("Stop Simulation Program")
        self.program.Stop()
        self.inProgress = False
        sim_data = json.dumps(self.inProgress)
        print("sim_data: ", sim_data)

        # 예외 처리 추가
        try:
            self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
            self.robot1.MoveJ(self.home)
            self.rdk.setRunMode(RUNMODE_SIMULATE)
        except StoppedError as e:
            print(f"StoppedError: {e}")

        #self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
        #self.robot1.MoveJ(self.home)
        self.rdk.setRunMode(RUNMODE_SIMULATE)
        await websocket.send(sim_data)

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

            reachable1 = self.reachable1

            if self.isCollision:
                collision_detection = json.dumps("collision_detection")
                #self.isCollision = False
                print("self.isCollision: ", self.isCollision)
                await websocket.send(collision_detection)
            else:
                # 로봇 관절 각도 값들이나 턴테이블의 각도값이 변하였을 때 데이터 전송
                if not np.array_equal(current_joints1, self.previous_joints1):
                    # 웹소켓 전송 시간 시작
                    # print("send_joint_positions of robot[1]")
                    start_time3 = time.time()
                    joints_np = np.array(current_joints1)
                    joints_flat = joints_np.flatten()


                    joints_flat[0] = -joints_flat[0]

                    data = joints_flat.tolist()
                    data.append('robot1')
                    data.append(reachable1)
                    json_data = json.dumps(data)
                    print("sned data 2 WXR")
                    await websocket.send(json_data)

                    self.previous_joints1 = current_joints1

            # send_joint_positions에서 webSocket을 독점하는 것을 막기 위해
            await asyncio.sleep(0.001)
