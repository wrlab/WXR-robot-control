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
        self.robots = robots
        self.tools = tools
        self.ext_tools = ext_tools
        self.rdk = rdk

        # self.robot1 = robots[0]
        # self.tool1 = tools[0]
        # self.turntable1 = ext_tools[0]
        # self.robot2 = robots[1]
        # self.tool2 = tools[1]
        # self.turntable2 = ext_tools[1]

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

        # Profile에 대한 정보 확인
        self.log_robot_profile_info()

        # teleoperation을 위한 변수
        self.current_joints_ghost = self.robots[0].SimulatorJoints()
        self.previous_joints_ghost = None
        # self.current_joints_shadow = self.robots[1].SinulatorJoints()
        # self.previous_joints_shadow = None

        # 원격 제어 on/off 변수
        self.isTeleoper = True
        self.current_mode = 'Remote'
        # 충돌 감지 및 가동범위 변수
        self.isCollision = False
        self.reachable = True
        # log 출력을 위한 변수
        self.num = 0
        self.send_num = 0
        # 코루틴 Task 핸들
        self.send_task = None # 현재 동작 중인 (teleop/mirror) 전송 코루딘
        # 포즈 변화 임계값
        self.position_threshold = 10 # position 임계값 설정 (단위: mm)
        self.rotation_threshold = 1 # rotation 임계값 설정 (단위: degree)
        # 이벤트 기반 처리를 위한 큐
        self.command_queue = asyncio.Queue()
        self.update_event = asyncio.Event()
        self.rdkapi_math = rdkapiMath()

        ## 2025.01.17 기준 나중에 활용할 기능에 대한 변수
        self.program = self.rdk.Item("Prog1")
        self.inProgress = False
        self.home = self.rdk.Item("Home2")

        # 폴더 생성
        self.init_directories()

        # 로그 파일 설정
        self.buffer_log_file = self.get_unique_log_file("points/pose_log.txt")
        self.setup_log_file()
        self.total_log_file = self.get_unique_log_file("points/total_pose_log.txt")
        self.setup_total_log_file()
        self.joints_log_file = self.get_unique_log_file("joints/joints_log.txt")
        self.setup_joints_log_file()

        self.tps = {}  # 모든 TP 데이터를 저장할 딕셔너리

        # 2번째 로봇에 대한 부분은 아직 구현x 24.03.08기준
        self.previous_joints1 = None
        self.previous_tables1 = None
        self.on_table1 = False

        # 2025.01.29
        self.inSafePose = False

        print("webSocket Start!")

    def log_robot_profile_info(self):
        print("[Robot profile Information]")
        for i, (robot, tool, ext_tool) in enumerate(zip(self.robots, self.tools, self.ext_tools)):
            print(f"Robot {i + 1}: {robot.Name()}")
            print(f"  Tool: {tool.Name()}")
            print(f"  External Tool: {ext_tool.Name()}")

    def init_directories(self):
        """Create necessary directories if they do not exist."""
        if not os.path.exists("points"):
            os.makedirs("points")
        if not os.path.exists("joints"):
            os.makedirs("joints")

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
        # 처음 들어올 때 기본 모드(Remote)라면 teleop_send 먼저 할당
        if self.current_mode == "Remote":
            self.send_task = asyncio.create_task(self.teleop_send(websocket))
        else:
            self.send_task = asyncio.create_task(self.mirror_send(websocket))
        sender_task = self.send_task
        operation_task = asyncio.create_task(self.tool_teleoperation(1))

        await asyncio.gather(receiver_task, sender_task, operation_task)

    async def tool_teleoperation(self, robot_id):
        #robot = getattr(self, f'robot{robot_id}')
        robot = self.robots[0]
        tool = self.tools[0]
        positioner = self.ext_tools[0]
        print("tool_teleoperation Start!")

        # 2024.12.18
        last_req_pose = None
        last_req_positioner = None

        while True:
            data = await self.command_queue.get()

            if isinstance(data, tuple) and len(data) == 2:
                position, rotation = data
                #print("rotation: ", rotation)

                if self.isTeleoper and position and rotation:
                    new_pose = self.rdkapi_math.cal_local_pose(position, rotation)
                    current_joints = self.rdkapi_math.cal_ik(robot, new_pose, tool)
                    #print("new_pose: ", new_pose)

                    if current_joints is None:
                        print("There is NO joints")
                        continue

                    if last_req_pose is not None:
                        pos_diff = self.rdkapi_math.position_dif(last_req_pose, new_pose)
                        rot_diff = self.rotation_dif(last_req_pose, new_pose)
                        if pos_diff > self.position_threshold or rot_diff > self.rotation_threshold:
                            self.execute_move_joints(robot, current_joints, new_pose)
                            last_req_pose = new_pose
                            self.update_event.set()
                            await asyncio.sleep(0)
                        else:
                            pass
                    else:
                        self.execute_move_joints(robot, current_joints, new_pose)
                        last_req_pose = new_pose
                        self.update_event.set()
                        await asyncio.sleep(0)

            elif isinstance(data, dict) and "a_val" in data and "b_val" in data:
                # 포지셔너 데이터를 처리
                a_val = data["a_val"]
                b_val = data["b_val"]
                positioner_joints = [a_val, b_val]  # 포지셔너 축값
                if last_req_positioner is None or not np.array_equal(positioner_joints, last_req_positioner):
                    # RoboDK 포지셔너 모델의 값 변경
                    positioner.MoveJ(positioner_joints)
                    if self.isRealMode:
                        # RoboDK 포지셔너의 변화값을 6축+2축으로 하여 KUKA 로봇에 전달
                        self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
                        current_joints = self.robots[0].Joints().list()
                        current_joints.extend(positioner_joints)
                        self.robots[0].MoveJ(current_joints)

                    last_req_positioner = positioner_joints
                    #self.update_event.set()
                    #print(f"[tool_teleoperation] Positioner moved to: {positioner_joints}")
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

    def execute_move_joints(self, robot, joints, new_pose, websocket=None):
        # 만약 이미 충돌 상태이면, 추가 명령을 무시하도록 설정
        if self.isCollision:
            print("[execute_move_joints] Already in collision state, skipping MoveJ command.")
            return

        try:
            if self.isRealMode:
                self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
            else:
                self.rdk.setRunMode(RUNMODE_SIMULATE)

            # MoveJ 실행
            print("Joints: ", joints)
            robot.MoveJ(joints)
            self.num += 1
            print(f"[tool_teleoperation] Execute MoveJ #{self.num}")
            new_pose_kuka = Pose_2_KUKA(new_pose)
            self.log_pose(new_pose.Pos(), [new_pose_kuka[3], new_pose_kuka[4], new_pose_kuka[5]])
            self.log_joints(robot.Joints().list())

        except Exception as e:
            # 충돌 발생 시 처리
            if "Collision detected" in str(e):
                print("[execute_move_joints] Collision detected!", e)
                self.isCollision = True
                # 충돌 시점을 로그로 남기고 싶다면
                # self.log_collision_event()
                # 이후 return으로 함수 종료
                return
            else:
                # 다른 예외는 그대로 다시 raise
                raise

    # 2025.01.03 kuka 시스템 변수 읽기 위한 메소드 추가
    def read_system_variable(self, kuka_var_name):
        """
        실제 KUKA 로봇에 연결된 경우, 시스템 변수를 읽어오는 예시.
        RoboDK가 실 로봇 드라이버와 연결되어 있어야 하며, RUNMODE_RUN_ROBOT 모드에서만 정상적으로 읽힘.
        """
        old_mode = self.rdk.RunMode()  # 기존 모드 저장
        self.rdk.setRunMode(RUNMODE_RUN_ROBOT)  # 실제 로봇 모드로 전환

        # "Driver" 파라미터에 "GET [KUKA 변수명]"을 전달하면 해당 변수를 읽어옴
        value = self.robots[0].setParam("Driver", f"GET {kuka_var_name}")
        print(f"[read_system_variable] KUKA {kuka_var_name} => {value}")

        # 모드 원상복귀
        self.rdk.setRunMode(old_mode)
        return value

    def switch_mode(self, new_mode, websocket):
        # 모드가 바뀔 때마다, 기존의 전송 코루틴을 취소하고 새 전송 코루틴 실행
        if new_mode == self.current_mode:
            print("[switch_mode} 이미 같은 모드: ", new_mode)
            return

        # 1) 이전 태스크 취소
        if self.send_task and not self.send_task.done():
            self.send_task.cancel()
            print("[switch_mode] Old send_task canceled.")

        # 2) 모드 플래그 업데이트
        self.current_mode = new_mode
        if new_mode == "Remote":
            self.isTeleoper = True
            # 3) 새 태스크: teleop
            self.send_task = asyncio.create_task(self.teleop_send(websocket))
        else:
            self.isTeleoper = False
            # 3) 새 태스크: mirroring
            self.send_task = asyncio.create_task(self.mirror_send(websocket))

    async def teleop_send(self, websocket):
        print("[teleop_send_positions] Started.")
        try:
            while True:
                # 이벤트가 set될 때까지 대기
                await self.update_event.wait()
                self.update_event.clear()

                # 로봇 관절 읽기
                current_joints1 = self.robots[0].Joints()
                reachable1 = self.reachable

                if self.isCollision:
                    collision_detection = json.dumps('collision_detection')
                    print("self.isCollision: ", self.isCollision)
                    await websocket.send(collision_detection)
                else:
                    # 로봇 관절 각도 값들이나 턴테이블의 각도값이 변하였을 때 데이터 전송
                    if self.previous_joints1 is None or not np.array_equal(current_joints1, self.previous_joints1):
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

        except asyncio.CancelledError:
            print("[teleop_send_positions] Cancelled.")
        except websockets.exceptions.ConnectionClosed:
            print("[teleop_send_positions] Connection closed.")
        except Exception as e:
            print("[teleop_send_positions] Exception:", e)

    async def mirror_send(self, websocket):
        print("[mirror_send_positions] Started.")
        try:
            while True:
                current_joints1 = self.robots[0].Joints()
                reachable1 = self.reachable

                # 현재 로봇 포즈 가져오기
                reference_pose = self.robots[0].Pose()
                position = reference_pose.Pos()  # 포지션 [x, y, z]
                kuka_pose = Pose_2_KUKA(reference_pose)
                rotation = [kuka_pose[3], kuka_pose[4], kuka_pose[5]]  # 로테이션 [a, b, c]

                if self.previous_joints1 is None or not np.array_equal(current_joints1, self.previous_joints1):
                    joints_np = np.array(current_joints1)
                    joints_flat = joints_np.flatten().tolist()
                    joints_flat.append('robot1')
                    joints_flat.append(reachable1)

                    # 포지션과 로테이션 추가
                    joints_flat.append({"position": position, "rotation": rotation})

                    # json 패킷
                    json_data = json.dumps(joints_flat)
                    self.send_num += 1
                    print("Immediate state update:", self.send_num)
                    await websocket.send(json_data)
                    self.previous_joints1 = current_joints1

                await asyncio.sleep(0.0001)

        except asyncio.CancelledError:
            print("[mirror_send_positions] Cancelled.")
        except websockets.exceptions.ConnectionClosed:
            print("[mirror_send_positions] Connection closed.")
        except Exception as e:
            print("[mirror_send_positions] Exception:", e)

    async def receive_messages(self, websocket):
        async for message in websocket:
            data = json.loads(message)

            if data.get("command") == "start_streaming":
                print("Start streaming command received")

            if data.get("command") == "onoff_turntable":
                self.on_table1 = data.get("onoff")

            if data.get("command") == "update_position":
                # 2025.01.29
                if self.inSafePose:
                    print("[receive_messages] Ignoring TCP move because inSafePose=True")
                    continue

                position = data.get("position")
                rotation = data.get("rotation")
                #print("update_position Rotation: ", rotation)
                new_pose = self.rdkapi_math.cal_local_pose(position, rotation)
                self.log_total_pose(new_pose.Pos(), rotation)
                # 이벤트 기반으로 명령 enqueue
                await self.command_queue.put((position, rotation))

            if data.get("command") == "update_positioner":
                a_axis = data.get("aAxis", {})
                b_axis = data.get("bAxis", {})
                a_val = round(a_axis.get("e1", 0.0), 5)
                b_val = round(b_axis.get("e2", 0.0), 5)
                await self.command_queue.put({"a_val": a_val, "b_val": b_val})

            # 2025.01.29
            if data.get("command") == "return_home":
                print("[receive_messages] Return to safe pose.")
                self.isCollision = False
                self.rdk.setCollisionActive(0)

                self.inSafePose = True

                safe_pose_target = self.rdk.Item('safe-pose', ITEM_TYPE_TARGET)
                if safe_pose_target.Valid():
                    safe_pose = safe_pose_target.Pose()
                    self.robots[0].MoveJ(safe_pose)
                    print("Robot moved to safe-pose.")

                    current_joints1 = self.robots[0].Joints()
                    reachable1 = self.reachable

                    # 현재 로봇 포즈 가져오기
                    reference_pose = self.robots[0].Pose()
                    position = reference_pose.Pos()  # 포지션 [x, y, z]
                    kuka_pose = Pose_2_KUKA(reference_pose)
                    rotation = [kuka_pose[3], kuka_pose[4], kuka_pose[5]]  # 로테이션 [a, b, c]

                    # 웹소켓 전송 시간 시작
                    joints_np = np.array(current_joints1)
                    joints_flat = joints_np.flatten().tolist()
                    joints_flat.append('robot1')
                    joints_flat.append(reachable1)

                    # 포지션과 로테이션 추가
                    joints_flat.append({"position": position, "rotation": rotation})

                    # json 패킷
                    json_data = json.dumps(joints_flat)
                    self.send_num += 1
                    print("Immediate state update:", self.send_num)
                    await websocket.send(json_data)
                    self.previous_joints1 = current_joints1
                else:
                    print("Target 'safe-pose' not found in RoboDK.")

            # 2025.01.27
            if data.get("command") == "resume_collision":
                print("[receive_messages] Collision resume command received.")
                self.isCollision = False
                self.rdk.setCollisionActive(1)

                self.inSafePose = False

            # 24.07.17 티칭포인트들 전달 받은 코드 추가
            if data.get("command") == "teaching_points":
                print("teaching points receiving")
                program_name = "teaching"
                count = 0
                created_targets = []
                program = self.rdk.AddProgram(program_name, self.robots[0])
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

                while self.robots[0].Busy():
                    await asyncio.sleep(0.5)  # 0.5초 간격으로 로봇 상태 체크

                program.Delete()
                for target in created_targets:
                    target.Delete()

            if data.get("command") == "setting mode":
                mode = data.get("mode")
                print("data: ", data)
                if mode == "Remote":
                    print("mode: ", mode)
                    self.switch_mode("Remote", websocket)
                elif mode == "Mirroring":
                    print("mode: ", mode)
                    self.switch_mode("Mirroring", websocket)

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
            self.robots[0].MoveJ(self.home)
            self.rdk.setRunMode(RUNMODE_SIMULATE)
        except StoppedError as e:
            print(f"StoppedError: {e}")

        self.rdk.setRunMode(RUNMODE_SIMULATE)
        await websocket.send(sim_data)
