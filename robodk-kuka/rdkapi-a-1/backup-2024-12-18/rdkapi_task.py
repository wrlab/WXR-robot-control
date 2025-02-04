import asyncio
import websockets
import numpy as np
import json
from robodk.robomath import *
from robodk.robolink import *
import time
import ssl
import os
from rdkapi_math import rdkapiMath


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

        self.rdk = rdk

        # 로봇 동작 관련 변수
        self.position1 = None  # 위치 데이터 초기화
        self.rotation1 = None  # 회전 데이터 초기화
        self.reachable1 = True  # IK솔루션 있을 경우 초기화

        self.current_joints = self.robot1.SimulatorJoints()

        # 원격 제어 on/off 변수
        self.isTeleoper = True
        self.program = self.rdk.Item("Prog1")
        # 시뮬레이션 진행에 대한 변수
        self.inProgress = False
        # home 포지션
        self.home = self.rdk.Item("Home2")
        # 충돌 여부에 대한 변수
        self.isCollision = False

        # 포즈 변화 임계값
        self.position_threshold = 10  # position 임계값 설정 (단위: mm)
        self.rotation_threshold = 0.1  # rotation 임계값 설정 (단위: degree)
        self.num = 0

        self.rdkapi_math = rdkapiMath()  # rdkapiMath 인스턴스 생성

        # Create the points directory if it doesn't exist
        if not os.path.exists("points"):
            os.makedirs("points")

        self.buffer_log_file = self.get_unique_log_file("points/pose_log.txt")
        self.setup_log_file()

        self.total_log_file = self.get_unique_log_file("points/total_pose_log.txt")
        self.setup_total_log_file()

        self.joints_log_file = self.get_unique_log_file("joints/joints_log.txt")
        self.setup_joints_log_file()

        self.tps = {}  # 모든 TP 데이터를 저장할 딕셔너리

        # 이벤트 기반 처리를 위한 큐
        self.command_queue = asyncio.Queue()

        print("webSocket Start!")

    def get_unique_log_file(self, base_path):
        counter = 1
        log_file = base_path
        while os.path.exists(log_file):
            log_file = f"{base_path.rstrip('.txt')}_{counter}.txt"
            counter += 1
        return log_file

    def setup_log_file(self):
        with open(self.buffer_log_file, 'w') as f:
            f.write("time,x,y,z,a,b,c\n")

    def setup_total_log_file(self):
        with open(self.total_log_file, 'w') as f:
            f.write("time,x,y,z,a,b,c\n")

    def setup_joints_log_file(self):
        with open(self.joints_log_file, 'w') as f:
            f.write("time,a1,a2,a3,a4,a5,a6\n")

    def log_pose(self, pos, rotation):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
        x = round(rotation[0], 3)
        y = round(rotation[1], 3)
        z = round(rotation[2], 3)
        with open(self.buffer_log_file, 'a') as f:
            f.write(f"{timestamp},{pos[0]},{pos[1]},{pos[2]}, {x}, {y}, {z}\n")

    def log_total_pose(self, pos, rotation):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
        # 소수점 3자리로 값 포맷
        x = round(rotation['x'], 3)
        y = round(rotation['y'], 3)
        z = round(rotation['z'], 3)

        with open(self.total_log_file, 'a') as f:
            f.write(f"{timestamp},{pos[0]},{pos[1]},{pos[2]}, {x}, {y}, {z}\n")

    def log_joints(self, joints):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
        with open(self.joints_log_file, 'a') as f:
            f.write(f"{timestamp},{joints[0]},{joints[1]},{joints[2]}, {joints[3]},{joints[4]},{joints[5]}\n")

    def start_server(self):
        server = websockets.serve(self.handler, self.host, self.port)
        asyncio.get_event_loop().run_until_complete(server)
        asyncio.get_event_loop().run_forever()

        # ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        # ssl_context.load_cert_chain(certfile="crt/livinglab/selfsigned.crt",
        #                             keyfile="crt/livinglab/selfsigned.key")
        # server = websockets.serve(self.handler, self.host, self.port, ssl=ssl_context)
        #
        # asyncio.get_event_loop().run_until_complete(server)
        # asyncio.get_event_loop().run_forever()

    def make_teachingProg(self, data):
        for tp in data.get("tps", []):
            self.tps[tp['id']] = {
                'position': tp['position'],
                'rotation': tp['rotation']
            }
            print(f"{tp['id']} - Position: {tp['position']}, Rotation: {tp['rotation']}")

    async def handler(self, websocket):
        receiver_task = asyncio.create_task(self.receive_messages(websocket))
        sender_task = asyncio.create_task(self.send_joint_positions(websocket))
        operation_task = asyncio.create_task(self.tool_teleoperation(1))

        await asyncio.gather(receiver_task, sender_task, operation_task)

    async def tool_teleoperation(self, robot_id):
        robot = getattr(self, f'robot{robot_id}')
        print("tool_teleoperation Start!")

        # 2024.12.18
        last_req_pose = None

        while True:
            # command_queue에서 새로운 position/rotation 데이터를 대기
            position, rotation = await self.command_queue.get()

            if self.isTeleoper and position and rotation:
                current_pose = robot.Pose()
                new_pose = self.rdkapi_math.cal_local_pose(position, rotation)

                # 회전 차이 계산
                tool_pose = Pose_2_KUKA(current_pose)
                tool_rot = tool_pose[3:]
                tool_rot_dict = {'x': tool_rot[0], 'y': tool_rot[1], 'z': tool_rot[2]}
                rotation_matrix = self.rdkapi_math.euler_to_matrix(tool_rot_dict)
                new_rot_matrix = self.rdkapi_math.euler_to_matrix(rotation)

                # rotation_matrix = self.rdkapi_math.euler_to_matrix(tool_rot_dict)
                # print("rotation: ", rotation)
                # new_rot_dict = {'x': rotation[0], 'y': rotation[1], 'z': rotation[2]}
                # new_rot_matrix = self.rdkapi_math.euler_to_matrix(new_rot_dict)

                pos_diff = self.rdkapi_math.position_dif(current_pose, new_pose)
                rot_diff = self.rdkapi_math.rotation_matrix_difference(rotation_matrix, new_rot_matrix)

                # if (self.rdkapi_math.pose_dif(current_pose, new_pose) > self.position_threshold
                #         or rot_diff > self.rotation_threshold):
                if pos_diff > self.position_threshold or rot_diff > self.rotation_threshold:
                    #print("new_pose.Pos():", new_pose.Pos())
                    robot.MoveJ(new_pose)
                    self.num += 1
                    print("MoveJ", self.num)
                    self.log_pose(new_pose.Pos(), tool_rot)
                    self.log_joints(robot.Joints().list())
                # 여기서 추가적인 sleep은 필요 없음. 다음 명령이 들어올 때까지 대기.

    async def receive_messages(self, websocket):
        async for message in websocket:
            data = json.loads(message)

            if data.get("command") == "start_streaming":
                print("Start streaming command received")

            if data.get("command") == "onoff_turntable":
                self.on_table1 = data.get("onoff")

            if data.get("command") == "update_position":
                position = data.get("position")
                rotation = data.get("rotation")
                new_pose = self.rdkapi_math.cal_local_pose(position, rotation)
                self.log_total_pose(new_pose.Pos(), rotation)
                #print("position: ", position)
                #print("rotation: ", rotation)
                # 이벤트 기반으로 명령 enqueue
                await self.command_queue.put((position, rotation))

            # 24.07.17 티칭포인트들 전달 받은 코드 추가
            if data.get("command") == "teaching_points":
                print("teaching points receiving")
                program_name = "teaching"
                count = 0
                created_targets = []
                program = self.rdk.AddProgram(program_name, self.robot1)
                # self.make_teachingProg(data)
                for tp in data.get("tps", []):
                    self.tps[tp['id']] = {
                        'position': tp['position'],
                        'rotation': tp['rotation']
                    }
                    count = count + 1
                    points_pos = self.rdkapi_math.cal_local_pose(tp['position'], tp['rotation'])
                    target = self.rdk.AddTarget('T%i' % count)
                    target.setPose(points_pos)
                    program.MoveJ(target)
                    created_targets.append(target)
                    # self.robot1.MoveJ(points_pos)
                    print(f"{tp['id']} - Position: {tp['position']}, Rotation: {tp['rotation']}")

                program.RunProgram()

                while self.robot1.Busy():
                    await asyncio.sleep(0.5)  # 0.5초 간격으로 로봇 상태 체크

                program.Delete()
                for target in created_targets:
                    target.Delete()

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

            # await asyncio.sleep(0.001)

    async def stop_simulation(self, websocket):
        print("Stop Simulation Program")
        self.program.Stop()
        self.inProgress = False
        sim_data = json.dumps(self.inProgress)
        print("sim_data: ", sim_data)

        # 예외 처리 추가
        try:
            self.robot1.MoveJ(self.home)
            self.rdk.setRunMode(RUNMODE_SIMULATE)
        except StoppedError as e:
            print(f"StoppedError: {e}")

        self.rdk.setRunMode(RUNMODE_SIMULATE)
        await websocket.send(sim_data)

    async def send_joint_positions(self, websocket):
        print("send_joint_positions")
        while True:
            # 관절 위치 전송 로직
            current_joints1 = self.robot1.Joints()
            reachable1 = self.reachable1

            # 충돌 감지하였을 경우 충돌 감지 메시지 전달
            if self.isCollision:
                collision_detection = json.dumps("collision_detection")
                print("self.isCollision: ", self.isCollision)
                await websocket.send(collision_detection)
            else:
                # 로봇 관절 각도 값들이나 턴테이블의 각도값이 변하였을 때 데이터 전송
                if not np.array_equal(current_joints1, self.previous_joints1):
                    # 웹소켓 전송 시간 시작
                    joints_np = np.array(current_joints1)
                    joints_flat = joints_np.flatten().tolist()
                    joints_flat.append('robot1')
                    joints_flat.append(reachable1)

                    # json 패킷
                    json_data = json.dumps(joints_flat)
                    print("sned data 2 WXR")
                    await websocket.send(json_data)
                    self.previous_joints1 = current_joints1

            # send_joint_positions에서 webSocket을 독점하는 것을 막기 위해
            await asyncio.sleep(0.01)
