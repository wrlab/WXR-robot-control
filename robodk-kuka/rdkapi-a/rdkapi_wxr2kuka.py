import asyncio
import websockets
import numpy as np
import json
from robodk.robomath import *
from robodk.robolink import *
import time
import threading
import ssl
from rdkapi_math import rdkapiMath

socket_lock = threading.Lock()
socket_lock2 = threading.Lock()
pose_lock = asyncio.Lock()

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

        # IK 계산용 thread 생성 [robot1]
        self.position1 = None  # 위치 데이터 초기화
        self.rotation1 = None  # 회전 데이터 초기화
        self.reachable1 = True  # IK솔루션 있을 경우 초기화
        #self.reachable_event1 = asyncio.Event()

        self.current_joints = self.robot1.SimulatorJoints()

        # default smooth parameters
        self.Ts = 0.01 # sampling 주기
        self.kp3 = 30 # 3단계 위치 보정
        #self.kp3 = 2  # 3단계 위치 보정
        self.kd3 = 4.2 # 3단계 속도 보정
        self.u3_plus = 1.0 # 3단계 슬라이딩 모드 스위칭
        #self.u3_plus = 0.01  # 3단계 슬라이딩 모드 스위칭

        # 원격 제어 on/off 변수
        self.isTeleoper = True
        #self.program = self.rdk.Item("Prog1")
        self.program = self.rdk.Item("Prog2")
        # 시뮬레이션 진행에 대한 변수
        self.inProgress = False
        # home 포지션
        self.home = self.rdk.Item("Home2")
        # 충돌 여부에 대한 변수
        self.isCollision = False

        # 24.06.11 - new variables
        self.current_pose = self.robot1.Pose()
        self.threshold = 0.1 # 임계값 설정 (단위: mm)
        self.num = 0
        self.command_queue = asyncio.Queue()
        
        self.rdkapi_math = rdkapiMath() # rdkapiMath 인스턴스 생성

        # Create the points directory if it doesn't exist
        if not os.path.exists("points"):
            os.makedirs("points")

        self.buffer_log_file = self.get_unique_log_file("points/pose_log.txt")
        self.setup_log_file()

        self.total_log_file = self.get_unique_log_file("points/total_pose_log.txt")
        self.setup_total_log_file()

        self.tps = {} # 모든 TP 데이터를 저장할 딕셔너리


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
            f.write("time,x,y,z\n")

    def setup_total_log_file(self):
        with open(self.total_log_file, 'w') as f:
            f.write("time,x,y,z\n")

    def log_pose(self, pos):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
        with open(self.buffer_log_file, 'a') as f:
            f.write(f"{timestamp},{pos[0]},{pos[1]},{pos[2]}\n")

    def log_total_pose(self, pos):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
        with open(self.total_log_file, 'a') as f:
            f.write(f"{timestamp},{pos[0]},{pos[1]},{pos[2]}\n")

    def start_server(self):
        #ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        #ssl_context.load_cert_chain(certfile="C:/GitProjects/robodk-kuka/wxr2rdk-smooth-motion/selfsigned.crt", keyfile="C:/GitProjects/robodk-kuka/wxr2rdk-smooth-motion/selfsigned.key")
        #server = websockets.serve(self.handler, self.host, self.port, ssl=ssl_context)
        server = websockets.serve(self.handler, self.host, self.port)
        asyncio.get_event_loop().run_until_complete(server)
        asyncio.get_event_loop().run_forever()

    async def handler(self, websocket):
        receiver_task = asyncio.create_task(self.receive_messages(websocket))
        sender_task = asyncio.create_task(self.send_joint_positions(websocket))
        operation_task = asyncio.create_task(self.tool_teleoperation(1))

        await asyncio.gather(receiver_task, sender_task, operation_task)
    async def tool_teleoperation(self, robot_id):
        while True:
            if (self.isTeleoper):
                robot = getattr(self, f'robot{robot_id}')
                tool = getattr(self, f'tool{robot_id}')
                # 업데이트된 포지션과 로테이션값
                position = getattr(self, f'position{robot_id}')
                rotation = getattr(self, f'rotation{robot_id}')

                if position and rotation:
                    current_pose = robot.Pose()
                    new_pose = self.rdkapi_math.cal_local_pose(position, rotation)

                    # print("new_pose:", new_pose.Pos())
                    # robot.MoveJ(new_pose)
                    # self.num += 1
                    # print("MoveJ", self.num)
                    # self.log_pose(new_pose.Pos())
                    # self.rdk.setRunMode(RUNMODE_SIMULATE)

                    if self.rdkapi_math.pose_dif(current_pose, new_pose) > self.threshold:
                        print("new_pose:",new_pose.Pos())
                        robot.MoveJ(new_pose)
                        self.num += 1
                        print("MoveJ", self.num)
                        self.log_pose(new_pose.Pos())
                        #self.rdk.setRunMode(RUNMODE_SIMULATE)

                await asyncio.sleep(0.001)
            else:
                await asyncio.sleep(0.001)

    async def receive_messages(self, websocket):
        async for message in websocket:
            data = json.loads(message)
            #print("data: ", data)

            if data.get("command") == "start_streaming":
                print("Start streaming command received")

            if data.get("command") == "onoff_turntable":
                # self.on_table True 또는 False 받기
                self.on_table1 = data.get("onoff")

            if data.get("command") == "update_position":
                self.position1 = data.get("position")
                self.rotation1 = data.get("rotation")
                new_pose = self.rdkapi_math.cal_local_pose(self.position1, self.rotation1)
                self.log_total_pose(new_pose.Pos())
                print("position1: ", self.position1)

            # 24.07.17 티칭포인트들 전달 받은 코드 추가
            if data.get("command") == "teaching_points":
                for tp in data.get("tps", []):
                    self.tps[tp['id']] = {
                        'position': tp['position'],
                        'rotation': tp['rotation']
                    }
                    print(f"{tp['id']} - Position: {tp['position']}, Rotation: {tp['rotation']}")

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

            #await asyncio.sleep(0.001)

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
                    #print("joints_np: ", joints_np)

                    joints_flat = joints_np.flatten()
                    data = joints_flat.tolist()
                    # robot1 에 대한 관절 각도 값
                    data.append('robot1')
                    data.append(reachable1)
                    # json 패킷
                    json_data = json.dumps(data)
                    print("sned data 2 WXR")
                    await websocket.send(json_data)

                    self.previous_joints1 = current_joints1

            # send_joint_positions에서 webSocket을 독점하는 것을 막기 위해
            await asyncio.sleep(0.001)
