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
import logging

# 로깅 기본 설정 (필요에 따라 logger.py 모듈로 분리할 수 있음)
logging.basicConfig(level=logging.INFO,
                    format='[%(asctime)s] %(levelname)s: %(message)s')


class WebSocketCommunication:
    def __init__(self, host, port, robots, tools, ext_tools, rdk, run_mode="1"):
        self.host = host
        self.port = port
        self.robots = robots
        self.tools = tools
        self.ext_tools = ext_tools
        self.rdk = rdk

        self.run_mode = run_mode
        self.isRealMode = (self.run_mode == "2")
        logging.info("[INIT] %s mode enabled.", "Real" if self.isRealMode else "Simulation")

        self.log_robot_profile_info()

        # 원격 제어 관련 변수들
        self.isTeleoper = True
        self.current_mode = 'Remote'
        self.isCollision = False
        self.reachable = True

        # 로깅 및 상태 업데이트 카운터
        self.num = 0
        self.send_num = 0

        # 비동기 태스크 및 이벤트
        self.send_task = None
        self.command_queue = asyncio.Queue()
        self.update_event = asyncio.Event()

        # 포즈/관절 임계값
        self.position_threshold = 10  # mm
        self.rotation_threshold = 1  # degree

        self.rdkapi_math = rdkapiMath()

        # 프로그램 및 안전 포즈 관련 변수
        self.program = self.rdk.Item("Prog1")
        self.inProgress = False
        self.home = self.rdk.Item("Home2")
        self.inSafePose = False

        # 로그 폴더 및 파일 생성
        self.init_directories()
        self.buffer_log_file = self.get_unique_log_file("points/pose_log.txt")
        self.setup_log_file()
        self.total_log_file = self.get_unique_log_file("points/total_pose_log.txt")
        self.setup_total_log_file()
        self.joints_log_file = self.get_unique_log_file("joints/joints_log.txt")
        self.setup_joints_log_file()


        # 코루틴 Task 핸들
        self.send_task = None # 현재 동작 중인 (teleop/mirror) 전송 코루딘
        # 포즈 변화 임계값
        self.position_threshold = 10 # position 임계값 설정 (단위: mm)
        self.rotation_threshold = 1 # rotation 임계값 설정 (단위: degree)

        self.tps = {}  # 티칭 포인트 저장 딕셔너리
        self.previous_joints1 = None
        self.on_table1 = False

        # teleoperation 용 ghost joints 등 (필요에 따라 삭제 또는 별도 관리)
        self.current_joints_ghost = self.robots[0].SimulatorJoints()
        self.previous_joints_ghost = None

        logging.info("WebSocketCommunication initialized.")

    # --------------------- 초기화 및 유틸리티 함수 ---------------------
    def log_robot_profile_info(self) -> None:
        logging.info("[Robot profile Information]")
        for i, (robot, tool, ext_tool) in enumerate(zip(self.robots, self.tools, self.ext_tools)):
            logging.info("Robot %d: %s", i + 1, robot.Name())
            logging.info("  Tool: %s", tool.Name())
            logging.info("  External Tool: %s", ext_tool.Name())

    def init_directories(self) -> None:
        for folder in ["points", "joints"]:
            if not os.path.exists(folder):
                os.makedirs(folder)

    def get_unique_log_file(self, base_path: str) -> str:
        counter = 1
        log_file = base_path
        while os.path.exists(log_file):
            log_file = f"{base_path.rstrip('.txt')}_{counter}.txt"
            counter += 1
        return log_file

    def setup_log_file(self) -> None:
        with open(self.buffer_log_file, 'w') as f:
            f.write("time,x,y,z,a,b,c\n")

    def setup_total_log_file(self) -> None:
        with open(self.total_log_file, 'w') as f:
            f.write("time,x,y,z,a,b,c\n")

    def setup_joints_log_file(self) -> None:
        with open(self.joints_log_file, 'w') as f:
            f.write("time,a1,a2,a3,a4,a5,a6\n")

    def log_pose(self, pos, rotation) -> None:
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
        x, y, z = [round(val, 3) for val in rotation]
        with open(self.buffer_log_file, 'a') as f:
            f.write(f"{timestamp},{pos[0]},{pos[1]},{pos[2]},{x},{y},{z}\n")

    def log_total_pose(self, pos, rotation: dict) -> None:
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
        x, y, z = [round(rotation[k], 3) for k in ('x', 'y', 'z')]
        with open(self.total_log_file, 'a') as f:
            f.write(f"{timestamp},{pos[0]},{pos[1]},{pos[2]},{x},{y},{z}\n")

    def log_joints(self, joints) -> None:
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
        with open(self.joints_log_file, 'a') as f:
            f.write(f"{timestamp},{','.join(str(j) for j in joints)}\n")


    # --------------------- 서버 시작 및 핸들러 ---------------------
    def start_server(self) -> None:
        server = websockets.serve(self.handler, self.host, self.port)
        loop = asyncio.get_event_loop()
        loop.run_until_complete(server)
        loop.run_forever()

        # ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        # ssl_context.load_cert_chain(certfile="crt/livinglab/selfsigned.crt",
        #                             keyfile="crt/livinglab/selfsigned.key")
        # server = websockets.serve(self.handler, self.host, self.port, ssl=ssl_context)
        #
        # asyncio.get_event_loop().run_until_complete(server)
        # asyncio.get_event_loop().run_forever()

    async def handler(self, websocket) -> None:
        receiver_task = asyncio.create_task(self.receive_messages(websocket))
        # 초기 모드에 따라 전송 태스크 생성
        if self.current_mode == "Remote":
            self.send_task = asyncio.create_task(self.teleop_send(websocket))
        else:
            self.send_task = asyncio.create_task(self.mirror_send(websocket))
        # 로봇 툴 teleoperation 태스크
        teleop_task = asyncio.create_task(self.tool_teleoperation(1))
        await asyncio.gather(receiver_task, self.send_task, teleop_task)

    def switch_mode(self, new_mode: str, websocket) -> None:
        if new_mode == self.current_mode:
            logging.info("이미 같은 모드: %s", new_mode)
            return

        if self.send_task and not self.send_task.done():
            self.send_task.cancel()
            logging.info("이전 send_task 취소됨.")

        self.current_mode = new_mode
        self.isTeleoper = (new_mode == "Remote")

        if self.isTeleoper:
            self.send_task = asyncio.create_task(self.teleop_send(websocket))
        else:
            self.send_task = asyncio.create_task(self.mirror_send(websocket))

    # --------------------- Teleoperation 관련 ---------------------
    async def tool_teleoperation(self, robot_id: int) -> None:
        robot = self.robots[0]
        tool = self.tools[0]
        positioner = self.ext_tools[0]
        logging.info("tool_teleoperation started.")
        last_req_pose = None
        last_req_positioner = None

        while True:
            data = await self.command_queue.get()
            if isinstance(data, tuple) and len(data) == 2:
                position, rotation = data
                if self.isTeleoper and position and rotation:
                    new_pose = self.rdkapi_math.cal_local_pose(position, rotation)
                    current_joints = self.rdkapi_math.cal_ik(robot, new_pose, tool)
                    if current_joints is None:
                        logging.warning("No IK solution found.")
                        continue
                    if last_req_pose is not None:
                        pos_diff = self.rdkapi_math.position_dif(last_req_pose, new_pose)
                        rot_diff = self.rotation_dif(last_req_pose, new_pose)
                        if pos_diff > self.position_threshold or rot_diff > self.rotation_threshold:
                            self.execute_move_joints(robot, current_joints, new_pose)
                            last_req_pose = new_pose
                            self.update_event.set()
                        # else: 변화가 작으면 이동하지 않음
                    else:
                        self.execute_move_joints(robot, current_joints, new_pose)
                        last_req_pose = new_pose
                        self.update_event.set()
            elif isinstance(data, dict) and "a_val" in data and "b_val" in data:
                # 포지셔너 데이터 처리
                a_val = data["a_val"]
                b_val = data["b_val"]
                positioner_joints = [a_val, b_val]
                if last_req_positioner is None or not np.array_equal(positioner_joints, last_req_positioner):
                    positioner.MoveJ(positioner_joints)
                    if self.isRealMode:
                        self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
                        current_joints = self.robots[0].Joints().list()
                        current_joints.extend(positioner_joints)
                        self.robots[0].MoveJ(current_joints)
                    last_req_positioner = positioner_joints
            await asyncio.sleep(0)

    def rotation_dif(self, pose_old, pose_new) -> float:
        old_pose_kuka = Pose_2_KUKA(pose_old)
        new_pose_kuka = Pose_2_KUKA(pose_new)
        old_rot = {'x': old_pose_kuka[3], 'y': old_pose_kuka[4], 'z': old_pose_kuka[5]}
        new_rot = {'x': new_pose_kuka[3], 'y': new_pose_kuka[4], 'z': new_pose_kuka[5]}
        R_old = self.rdkapi_math.euler_to_matrix(old_rot)
        R_new = self.rdkapi_math.euler_to_matrix(new_rot)
        return self.rdkapi_math.rotation_matrix_difference(R_old, R_new)

    def execute_move_joints(self, robot, joints, new_pose, websocket=None) -> None:
        if self.isCollision:
            logging.warning("Collision state active: MoveJ command skipped.")
            return
        try:
            self.rdk.setRunMode(RUNMODE_RUN_ROBOT if self.isRealMode else RUNMODE_SIMULATE)
            robot.MoveJ(joints)
            self.num += 1
            logging.info("Execute MoveJ #%d", self.num)
            new_pose_kuka = Pose_2_KUKA(new_pose)
            self.log_pose(new_pose.Pos(), [new_pose_kuka[3], new_pose_kuka[4], new_pose_kuka[5]])
            self.log_joints(robot.Joints().list())
        except Exception as e:
            if "Collision detected" in str(e):
                logging.error("Collision detected: %s", e)
                self.isCollision = True
                return
            else:
                raise

    def read_system_variable(self, kuka_var_name: str):
        old_mode = self.rdk.RunMode()
        self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
        value = self.robots[0].setParam("Driver", f"GET {kuka_var_name}")
        logging.info("KUKA %s => %s", kuka_var_name, value)
        self.rdk.setRunMode(old_mode)
        return value


    # --------------------- 전송 관련 함수 ---------------------
    async def teleop_send(self, websocket) -> None:
        logging.info("teleop_send started.")
        try:
            while True:
                await self.update_event.wait()
                self.update_event.clear()
                current_joints1 = self.robots[0].Joints()
                if self.isCollision:
                    await websocket.send(json.dumps('collision_detection'))
                else:
                    if self.previous_joints1 is None or not np.array_equal(current_joints1, self.previous_joints1):
                        joints_np = np.array(current_joints1)
                        joints_flat = joints_np.flatten().tolist()
                        joints_flat.extend(['robot1', self.reachable])
                        json_data = json.dumps(joints_flat)
                        self.send_num += 1
                        logging.info("Immediate state update: %d", self.send_num)
                        await websocket.send(json_data)
                        self.previous_joints1 = current_joints1
                if self.isRealMode:
                    self.read_system_variable("$PRO_MOVE")
                    self.read_system_variable("$AXIS_ACT")
        except asyncio.CancelledError:
            logging.info("teleop_send cancelled.")
        except websockets.exceptions.ConnectionClosed:
            logging.info("teleop_send: Connection closed.")
        except Exception as e:
            logging.exception("teleop_send exception: %s", e)

    async def mirror_send(self, websocket) -> None:
        logging.info("mirror_send started.")
        try:
            while True:
                current_joints1 = self.robots[0].Joints()
                reference_pose = self.robots[0].Pose()
                position = reference_pose.Pos()
                kuka_pose = Pose_2_KUKA(reference_pose)
                rotation = [kuka_pose[3], kuka_pose[4], kuka_pose[5]]
                if self.previous_joints1 is None or not np.array_equal(current_joints1, self.previous_joints1):
                    joints_np = np.array(current_joints1)
                    joints_flat = joints_np.flatten().tolist()
                    joints_flat.extend(['robot1', self.reachable])
                    joints_flat.append({"position": position, "rotation": rotation})
                    json_data = json.dumps(joints_flat)
                    self.send_num += 1
                    logging.info("Immediate state update: %d", self.send_num)
                    await websocket.send(json_data)
                    self.previous_joints1 = current_joints1
                await asyncio.sleep(0.0001)
        except asyncio.CancelledError:
            logging.info("mirror_send cancelled.")
        except websockets.exceptions.ConnectionClosed:
            logging.info("mirror_send: Connection closed.")
        except Exception as e:
            logging.exception("mirror_send exception: %s", e)

# --------------------- 메시지 수신 및 명령 디스패치 ---------------------
    async def receive_messages(self, websocket) -> None:
        async for message in websocket:
            data = json.loads(message)
            command = data.get("command")
            if command:
                await self.dispatch_command(command, data, websocket)
            # 추가적으로 다른 공통 처리 루틴이 있다면 여기에 추가

    async def dispatch_command(self, command: str, data: dict, websocket) -> None:
        # 명령어에 따른 처리 함수를 딕셔너리로 매핑하는 방식 사용
        command_handlers = {
            "start_streaming": self.handle_start_streaming,
            "onoff_turntable": self.handle_onoff_turntable,
            "update_position": self.handle_update_position,
            "update_positioner": self.handle_update_positioner,
            "return_home": self.handle_return_home,
            "resume_collision": self.handle_resume_collision,
            "teaching_points": self.handle_teaching_points,
            "setting mode": self.handle_setting_mode,
            "Simulation": self.handle_simulation,
            "change_realMode": self.handle_real_mode
        }
        handler = command_handlers.get(command)
        if handler:
            await handler(data, websocket)
        else:
            logging.warning("Unknown command received: %s", command)

# --------------------- 각 명령어별 핸들러 함수 ---------------------
    async def handle_start_streaming(self, data: dict, websocket) -> None:
        logging.info("Start streaming command received.")

    async def handle_onoff_turntable(self, data: dict, websocket) -> None:
        self.on_table1 = data.get("onoff")
        logging.info("Turntable on/off: %s", self.on_table1)

    async def handle_update_position(self, data: dict, websocket) -> None:
        if self.inSafePose:
            logging.info("Ignoring update_position because inSafePose is True.")
            return
        position = data.get("position")
        rotation = data.get("rotation")
        new_pose = self.rdkapi_math.cal_local_pose(position, rotation)
        self.log_total_pose(new_pose.Pos(), rotation)
        await self.command_queue.put((position, rotation))

    async def handle_update_positioner(self, data: dict, websocket) -> None:
        a_axis = data.get("aAxis", {})
        b_axis = data.get("bAxis", {})
        a_val = round(a_axis.get("e1", 0.0), 5)
        b_val = round(b_axis.get("e2", 0.0), 5)
        await self.command_queue.put({"a_val": a_val, "b_val": b_val})

    async def handle_return_home(self, data: dict, websocket) -> None:
        logging.info("Return to safe pose command received.")
        self.isCollision = False
        self.rdk.setCollisionActive(0)
        self.inSafePose = True
        safe_pose_target = self.rdk.Item('safe-pose', ITEM_TYPE_TARGET)
        if safe_pose_target.Valid():
            safe_pose = safe_pose_target.Pose()
            self.robots[0].MoveJ(safe_pose)
            logging.info("Robot moved to safe-pose.")
            # 전송 상태 업데이트 (중복되는 코드는 별도 함수로 분리 가능)
            current_joints1 = self.robots[0].Joints()
            reference_pose = self.robots[0].Pose()
            position = reference_pose.Pos()
            kuka_pose = Pose_2_KUKA(reference_pose)
            rotation = [kuka_pose[3], kuka_pose[4], kuka_pose[5]]
            joints_np = np.array(current_joints1)
            joints_flat = joints_np.flatten().tolist()
            joints_flat.extend(['robot1', self.reachable])
            joints_flat.append({"position": position, "rotation": rotation})
            json_data = json.dumps(joints_flat)
            self.send_num += 1
            logging.info("Immediate state update: %d", self.send_num)
            await websocket.send(json_data)
            self.previous_joints1 = current_joints1
        else:
            logging.error("Target 'safe-pose' not found in RoboDK.")

    async def handle_resume_collision(self, data: dict, websocket) -> None:
        logging.info("Resume collision command received.")
        self.isCollision = False
        self.rdk.setCollisionActive(1)
        self.inSafePose = False

    async def handle_teaching_points(self, data: dict, websocket) -> None:
        logging.info("Teaching points receiving.")
        program_name = "teaching"
        count = 0
        created_targets = []
        program = self.rdk.AddProgram(program_name, self.robots[0])
        for tp in data.get("tps", []):
            self.tps[tp['id']] = {'position': tp['position'], 'rotation': tp['rotation']}
            count += 1
            points_pos = self.rdkapi_math.cal_local_pose(tp['position'], tp['rotation'])
            target = self.rdk.AddTarget(f"T{count}")
            target.setPose(points_pos)
            program.MoveJ(target)
            created_targets.append(target)
            logging.info("Teaching point %s: Position %s, Rotation %s", tp['id'], tp['position'], tp['rotation'])
        program.RunProgram()
        while self.robots[0].Busy():
            await asyncio.sleep(0.5)
        program.Delete()
        for target in created_targets:
            target.Delete()

    async def handle_setting_mode(self, data: dict, websocket) -> None:
        mode = data.get("mode")
        logging.info("Setting mode to: %s", mode)
        if mode == "Remote":
            self.switch_mode("Remote", websocket)
        elif mode == "Mirroring":
            self.switch_mode("Mirroring", websocket)

    async def handle_simulation(self, data: dict, websocket) -> None:
        signal = data.get("signal")
        if signal == 'Start':
            logging.info("Start Simulation Program")
            self.program.RunProgram()
            self.rdk.setRunMode(RUNMODE_SIMULATE)
            self.inProgress = True
            sim_data = json.dumps(self.inProgress)
            await websocket.send(sim_data)
            while self.program.Busy():
                logging.info("Program is running...")
                await asyncio.sleep(0.5)
                try:
                    message = await asyncio.wait_for(websocket.recv(), timeout=0.001)
                    new_data = json.loads(message)
                    if new_data.get("command") == 'Simulation' and new_data.get("signal") == 'Stop':
                        await self.stop_simulation(websocket)
                        break
                except asyncio.TimeoutError:
                    continue
            self.inProgress = False
            sim_data = json.dumps(self.inProgress)
            sim_finished = json.dumps("sim_finished")
            await websocket.send(sim_data)
            await websocket.send(sim_finished)
        elif signal == 'Stop':
            await self.stop_simulation(websocket)

    async def stop_simulation(self, websocket) -> None:
        logging.info("Stop Simulation Program")
        self.program.Stop()
        self.inProgress = False
        sim_data = json.dumps(self.inProgress)
        try:
            self.robots[0].MoveJ(self.home)
            self.rdk.setRunMode(RUNMODE_SIMULATE)
        except StoppedError as e:
            logging.error("StoppedError: %s", e)
        self.rdk.setRunMode(RUNMODE_SIMULATE)
        await websocket.send(sim_data)

    async  def handle_real_mode(self, data: dict, websocket) -> None:
        logging.info("Toggling Real Mode State")

        # 현재 상태 반전
        self.isRealMode = not self.isRealMode
        logging.info("Real mode toggled. New state: %s", self.isRealMode)

        # 변경된 상태를 웹소켓을 통해 응답 메시지로 전달합니다.
        #response = {"command": "change_realMode", "real_mode": self.isRealMode}
        response = ["change_realMode", self.isRealMode]
        await websocket.send(json.dumps(response))
