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
    def __init__(self, host, port, robots, tools, ext_tools, rdk, run_mode="1"):
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

        # 2025.01.05 모드 설정
        # run_mode = "1" (시뮬레이션), "2" (실 로봇 제어)
        self.run_mode = run_mode
        # 실로봇 모드인지 여부를 Boolean으로 저장
        if self.run_mode == "2":
            self.isRealMode = True
            print("[INIT] Real mode enabled.")
        else:
            self.isRealMode = False
            print("[INIT] Simulation mode enabled.")

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
        self.current_mode = 'Remote'
        self.program = self.rdk.Item("Prog1")
        # 시뮬레이션 진행에 대한 변수
        self.inProgress = False
        # home 포지션
        self.home = self.rdk.Item("Home2")
        # 충돌 여부에 대한 변수
        self.isCollision = False
        self.num = 0
        self.send_num = 0

        # 코루틴 Task 핸들
        self.send_task = None # 현재 동작 중인 (teleop/mirror) 전송 코루딘

        # 포즈 변화 임계값
        self.position_threshold = 10  # position 임계값 설정 (단위: mm)
        self.rotation_threshold = 1  # rotation 임계값 설정 (단위: degree)

        # 이벤트 기반 처리를 위한 큐
        self.command_queue = asyncio.Queue()
        self.update_event = asyncio.Event()  # 이벤트 객체 생성

        self.rdkapi_math = rdkapiMath()  # rdkapiMath 인스턴스 생성

        # Create the points directory if it doesn't exist
        if not os.path.exists("points"):
            os.makedirs("points")

        self.buffer_log_file = self.get_unique_log_file("points/pose_log.txt")
        self.setup_log_file()

        self.total_log_file = self.get_unique_log_file("points/total_pose_log.txt")
        self.setup_total_log_file()

        # Joints 폴더도 필요하다는 가정 하에 만듦
        if not os.path.exists("joints"):
            os.makedirs("joints")

        self.joints_log_file = self.get_unique_log_file("joints/joints_log.txt")
        self.setup_joints_log_file()

        self.tps = {}  # 모든 TP 데이터를 저장할 딕셔너리

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
                new_pose = self.rdkapi_math.cal_local_pose(position, rotation)
                # 비교 기준: 이전에 명령을 낸 포즈(last_req_pose)가 있으면 그거랑 비교
                # 없으면(최초 명령) 바로 명령 실행
                if last_req_pose is not None:
                    # 위치 차이 계산
                    pos_diff = self.rdkapi_math.position_dif(last_req_pose, new_pose)
                    # 2025.01.02 수정
                    rot_diff = self.rotation_dif(last_req_pose, new_pose)
                    # 임계값 비교
                    if pos_diff > self.position_threshold or rot_diff > self.rotation_threshold:
                        self.execute_move(robot, new_pose)
                        # 충분한 변화가 있으므로 MoveJ 명령 발행
                        last_req_pose = new_pose
                        # 2025.01.02
                        # 이동 직후 즉시 상태 업데이트 요청
                        self.update_event.set()
                        await asyncio.sleep(0)
                    else:
                        # 변화가 미미하므로 명령 생략
                        pass
                else:
                    # 첫 요청이므로 바로 실행
                    self.execute_move(robot, new_pose)
                    # new_pose_kuka = Pose_2_KUKA(new_pose)
                    # self.log_pose(new_pose.Pos(), [new_pose_kuka[3], new_pose_kuka[4], new_pose_kuka[5]])
                    # self.log_joints(robot.Joints().list())
                    last_req_pose = new_pose
                    self.update_event.set()
                    await asyncio.sleep(0)

    # 2025.01.02 추가
    def rotation_dif(self, pose_old, pose_new):
        tool_pose = Pose_2_KUKA(pose_old)
        tool_rot_dict = {'x': tool_pose[3], 'y': tool_pose[4], 'z': tool_pose[5]}
        rotation_matrix_old = self.rdkapi_math.euler_to_matrix(tool_rot_dict)

        new_pose_kuka = Pose_2_KUKA(pose_new)
        new_rot_dict = {'x': new_pose_kuka[3], 'y': new_pose_kuka[4], 'z': new_pose_kuka[5]}
        rotation_matrix_new = self.rdkapi_math.euler_to_matrix(new_rot_dict)

        return self.rdkapi_math.rotation_matrix_difference(rotation_matrix_old, rotation_matrix_new)

    def execute_move(self, robot, new_pose):
        #self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
        robot.MoveJ(new_pose)
        self.rdk.setRunMode(RUNMODE_SIMULATE)
        self.num += 1
        print(f"[tool_teleoperation] Execute MoveJ #{self.num}")
        new_pose_kuka = Pose_2_KUKA(new_pose)
        self.log_pose(new_pose.Pos(), [new_pose_kuka[3], new_pose_kuka[4], new_pose_kuka[5]])
        self.log_joints(robot.Joints().list())

    # 2025.01.03 kuka 시스템 변수 읽기 위한 메소드 추가
    def read_system_variable(self, kuka_var_name):
        """
        실제 KUKA 로봇에 연결된 경우, 시스템 변수를 읽어오는 예시.
        RoboDK가 실 로봇 드라이버와 연결되어 있어야 하며, RUNMODE_RUN_ROBOT 모드에서만 정상적으로 읽힘.
        """
        old_mode = self.rdk.RunMode()  # 기존 모드 저장
        self.rdk.setRunMode(RUNMODE_RUN_ROBOT)  # 실제 로봇 모드로 전환

        # "Driver" 파라미터에 "GET [KUKA 변수명]"을 전달하면 해당 변수를 읽어옴
        value = self.robot1.setParam("Driver", f"GET {kuka_var_name}")
        print(f"[read_system_variable] KUKA {kuka_var_name} => {value}")

        # 모드 원상복귀
        self.rdk.setRunMode(old_mode)
        return value

    async def send_joint_positions(self, websocket):
        # 이 태스크는 주기적으로 상태를 전송하거나, 이벤트가 발생되면 즉시 전송

        while True:
            # 이벤트 대기: 이벤트가 설정되면 즉시 업데이트
            # 동시에 일정 주기로 업데이트하고 싶다면, asyncio.wait를 이용해 타이머 태스크와 이벤트를 병행 처리할 수 있음
            await self.update_event.wait()
            self.update_event.clear()

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
                    self.send_num += 1
                    print("Immediate state update:", self.send_num)
                    await websocket.send(json_data)
                    self.previous_joints1 = current_joints1

            if self.isRealMode:
                pro_move_val = self.read_system_variable("$PRO_MOVE")
                axis_act_val = self.read_system_variable("$AXIS_ACT")
                print("KUKA $PRO_MOVE :", pro_move_val)
                print("KUKA $AXIS_ACT :", axis_act_val)
        # # send_joint_positions에서 webSocket을 독점하는 것을 막기 위해
            # await asyncio.sleep(0.01)

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
                    self.isTeleoper = True
                    print("mode: ", mode)
                elif mode == "Mirroring":
                    self.isTeleoper = False
                    print("mode: ", mode)

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
