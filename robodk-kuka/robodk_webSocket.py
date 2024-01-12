import asyncio
import websockets
import numpy as np
import json
from robodk.robomath import *
import time
import threading

socket_lock = threading.Lock()


def rad2degree(rad):
    return rad * 180 / math.pi
class WebSocketCommunication:
    def __init__(self, host, port, robot, tool, turntable, rdk):
        self.host = host
        self.port = port
        self.robot = robot
        self.tool = tool
        self.turntable = turntable
        self.previous_joints = None
        self.previous_tables = None
        self.on_table = False
        self.rdk = rdk

        # IK 계산용 thread 생성
        self.position = None # 위치 데이터 초기화
        self.rotation = None # 회전 데이터 초기화
        self.reachable = True # IK솔루션 있을 경우 초기화
        self.ik_results = None
        self.ik_thread = threading.Thread(target=self.ik_cal_thread)
        self.ik_thread.start()
        print("webSocket Start!")


    def ik_cal_thread(self):
        while True:
            if self.position and self.rotation:
                #print("ik_cal Working!!")
                local_pose = KUKA_2_Pose([1000 * self.position['x'], -1000 * self.position['z'], 1000 * self.position['y'], self.rotation['x'], self.rotation['y'], self.rotation['z']])
                all_solutions = self.robot.SolveIK_All(local_pose, self.tool)

                # 2024.01.12 기준으로 if len(all_solutions) == 0: 이 부분으로 진입하지 않음 수정해야함!!
                if len(all_solutions) == 0:
                    print("IK unreachable!!")
                    # 현재는 메시지 출력으로 디버깅하고, 나중에는 단순 변수 변환으로 변경예정(2024.01.12)
                    #self.rdk.ShowMessage('There is no solution for requested position', True)
                    self.reachable = False
                else:
                    for j in all_solutions:
                        conf_RLF = self.robot.JointsConfig(j).list()

                        rear = conf_RLF[0]  # 1 if Rear , 0 if Front
                        lower = conf_RLF[1]  # 1 if Lower, 0 if Upper (elbow)
                        flip = conf_RLF[2]  # 1 if Flip , 0 if Non flip (Flip is usually when Joint 5 is negative)

                        if rear == 0 and lower == 0 and flip == 0:
                            self.ik_results = j
                            socket_lock.acquire()
                            self.robot.setJoints(self.ik_results[:6])
                            socket_lock.release()
                            # IK 계산이 완료된 후, position과 rotation을 None으로 설정
                            self.position = None
                            self.rotation = None
                            self.reachable = True
                            print("IK reachable!!")
                            break

            time.sleep(0.001)

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
                self.on_table = data.get("onoff")

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
                    self.turntable.setJoints(joint_values)
                elif (y1 == -180 and z1 == 180) or (y1 == 180 and z1 == -180):
                    joint_values = [-x1_degree, y2_degree]
                    self.turntable.setJoints(joint_values)
                else:
                    joint_values = [-x1_degree, y2_degree]
                    self.turntable.setJoints(joint_values)
                print(f"update_turntables")

            if data.get("command") == "update_position":
                print("Get update_position")
                #2024.01.02
                self.position = data.get("position")
                self.rotation = data.get("rotation")

    async def send_joint_positions(self, websocket):
        print("send_joint_positions")
        while True:
            # 관절 위치 전송 로직
            socket_lock.acquire()
            current_joints = self.robot.Joints()
            current_tables = self.turntable.Joints()
            reachable = self.reachable
            socket_lock.release()
            # 로봇 관절 각도 값들이나 턴테이블의 각도값이 변하였을 때 데이터 전송
            if not np.array_equal(current_joints, self.previous_joints):
                # 웹소켓 전송 시간 시작
                print("send_joint_positions")
                start_time3 = time.time()
                joints_np = np.array(current_joints)
                joints_flat = joints_np.flatten()

                data = joints_flat.tolist()
                json_data = json.dumps(data)
                await websocket.send(json_data)
                # 웹소켓 전송 시간 종료
                end_time3 = time.time()
                elapsed_time3 = end_time3 - start_time3
                print(f"변화감지 후 전송 시간: {elapsed_time3} 초")
                self.previous_joints = current_joints

            if not np.array_equal(current_tables, self.previous_tables) and not self.on_table:
                print("send_turntable_rotations")
                tables_np = np.array(current_tables)
                tables_flat = tables_np.flatten()
                data2 = tables_flat.tolist()
                json_data2 = json.dumps(data2)
                await websocket.send(json_data2)
                self.previous_tables = current_tables

            json_data3 = json.dumps(reachable)
            await  websocket.send(json_data3)

            # send_joint_positions에서 webSocket을 독점하는 것을 막기 위해
            await asyncio.sleep(0.00001)



