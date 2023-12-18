import asyncio
import websockets
import numpy as np
import json
from robodk.robomath import *


def rad2degree(rad):
    return rad * 180 / math.pi
class WebSocketCommunication:
    def __init__(self, host, port, robot, tool, turntable):
        self.host = host
        self.port = port
        self.robot = robot
        self.tool = tool
        self.turntable = turntable
        self.previous_joints = None
        self.previous_tables = None
        print("webSocket Start!")
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

            if data.get("command") == "update_turntables":
                joint1 = data.get("joint1")
                x1 = joint1.get("x")
                joint2 = data.get("joint2")
                y2 = joint2.get("y")
                # radian -> degree
                x1_degree = rad2degree(x1)
                y2_degree = rad2degree(y2)
                joint_values = [-x1_degree, y2_degree]
                self.turntable.setJoints(joint_values)

            if data.get("command") == "update_position":
                position = data.get("position")
                x = position.get("x")
                y = position.get("y")
                z = position.get("z")
                rotation = data.get("rotation")
                a = rotation.get("x")
                b = rotation.get("y")
                c = rotation.get("z")
                values = [1000 * x, -1000 * z, 1000 * y, a, b, c]
                local_pose = KUKA_2_Pose(values[:6])

                all_solutions = self.robot.SolveIK_All(local_pose, self.tool)
                for j in all_solutions:
                    conf_RLF = self.robot.JointsConfig(j).list()

                    rear = conf_RLF[0]  # 1 if Rear , 0 if Front
                    lower = conf_RLF[1]  # 1 if Lower, 0 if Upper (elbow)
                    flip = conf_RLF[2]  # 1 if Flip , 0 if Non flip (Flip is usually when Joint 5 is negative)

                    if rear == 0 and lower == 0 and flip == 0:
                        joints_sol = j
                        self.robot.setJoints(joints_sol[:6])
                        print(self.robot.Joints())
                        break

    async def send_joint_positions(self, websocket):
        print("send_joint_positions")
        while True:
            # 관절 위치 전송 로직
            current_joints = self.robot.Joints()
            current_tables = self.turntable.Joints()
            joints_np = np.array(current_joints)
            tables_np = np.array(current_tables)
            joints_flat = joints_np.flatten()
            tables_flat = tables_np.flatten()
            current_combined = np.concatenate((joints_flat, tables_flat))
            # 로봇 관절 각도 값들이나 턴테이블의 각도값이 변하였을 때 데이터 전송
            if not np.array_equal(current_joints, self.previous_joints) or not np.array_equal(current_tables, self.previous_tables):
                data = current_combined.tolist()
                json_data = json.dumps(data)
                await websocket.send(json_data)
                self.previous_joints = current_joints
                self.previous_tables = current_tables
            # send_joint_positions에서 webSocket을 독점하는 것을 막기 위해
            await asyncio.sleep(0.01)



