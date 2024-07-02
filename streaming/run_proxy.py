import logging
import time, json, math

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

import websockets
import asyncio
import ast

class SingletonMeta(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]
    
class RTDEService(metaclass=SingletonMeta):
    def __init__(self, ur_host_ip, ur_host_freq=125):
        self.ur_host_ip = ur_host_ip
        self.ur_host_freq = ur_host_freq
        self.receiver = RTDEReceive(ur_host_ip, ur_host_freq)
        self.controller = RTDEControl(ur_host_ip, ur_host_freq)
        
    def reconnect(self):
        self.controller.reconnect()
        self.receiver.reconnect()
        
    def disconnect(self):
        self.controller.disconnect()
        self.receiver.disconnect()
        
    def get_rtde_receive(self):
        return self.receiver
    
    def get_rtde_control(self):
        return self.controller


async def send_joint(websocket, joint_queue, stop_event):
    print(websocket, "send_joint")
    global args, service
    step_time_range = 1 / 125
    receiver = service.get_rtde_receive()
    controller = service.get_rtde_control()
    
    # Joint6의 돌기가 나가는 방향이 -y, 즉 반대방향이 +y
    # 마주보는 면에 뚫고 나가는 것이 +z, 
    # 왼손에서 엄지가 +z, 검지가 +y라면, +x는 중지가 나가는 방향
    
    while not stop_event.is_set():
        try:
            if not args.connection_test:
                joints = receiver.getActualQ()
                x, y, z, ax, ay, az = receiver.getActualTCPPose()
                temperatures = receiver.getJointTemperatures()
                
                print(controller.getTCPOffset())

            else:
                x, y, z = (
                    -0.12011317028458361,
                    -0.43176316676374304,
                    0.14607252291477707,
                )
                ax, ay, az = 0, 0, 0
                joints = [
                    0.0,
                    -1.5707963267948966,
                    1.5707963267948966,
                    0.0,
                    0.0,
                    0.0,
                ]

            await websocket.send(
                json.dumps(
                    {
                        "len": len(joints),
                        "q1": joints[0] * 180 / math.pi,
                        "q2": joints[1] * 180 / math.pi,
                        "q3": joints[2] * 180 / math.pi,
                        "q4": joints[3] * 180 / math.pi,
                        "q5": joints[4] * 180 / math.pi,
                        "q6": joints[5] * 180 / math.pi,
                        "x": x,
                        "y": y,
                        "z": z,
                        "ax": ax,
                        "ay": ay,
                        "az": az,
                    }
                ),
            )

            await joint_queue.put({
                        "len": len(joints),
                        "q1": joints[0] * 180 / math.pi,
                        "q2": joints[1] * 180 / math.pi,
                        "q3": joints[2] * 180 / math.pi,
                        "q4": joints[3] * 180 / math.pi,
                        "q5": joints[4] * 180 / math.pi,
                        "q6": joints[5] * 180 / math.pi,
                        "x": x,
                        "y": y,
                        "z": z,
                        "ax": ax,
                        "ay": ay,
                        "az": az,
                    })
            await asyncio.sleep(step_time_range)
        except websockets.ConnectionClosed:
            print("Connection closed in send_joint")
            stop_event.set()
            break
        except Exception as e:
            print(f"Exception in send_joint: {e}")
            stop_event.set()
            break


async def recv_tcp(websocket, tcp_queue, stop_event):
    print(websocket, "recv_tcp")
    while not stop_event.is_set():
        try:
            data = await websocket.recv()
            await tcp_queue.put(data)  # 큐에 데이터 추가
        except websockets.ConnectionClosed:
            print("fConnection closed in recv_tcp")
            stop_event.set()
            break
        except Exception as e:
            print(f"Exception in recv_tcp: {e}")
            stop_event.set()
            break

async def process_queues(tcp_queue, joint_queue, stop_event):  
    global args, service
    controller = service.get_rtde_control()
    
    start_time = time.time()
    while not stop_event.is_set():
        try:
            tcp_data_task = asyncio.create_task(tcp_queue.get())
            joint_data_task = asyncio.create_task(joint_queue.get())

            done, pending = await asyncio.wait(
                [tcp_data_task, joint_data_task],
                return_when=asyncio.FIRST_COMPLETED
            )
            
            if tcp_data_task in done:
                tcp_data = await tcp_data_task
                if tcp_data is not None:
                    tcp_data = json.loads(tcp_data)
                    x = tcp_data["tcp_pos"]["x"]
                    y = tcp_data["tcp_pos"]["y"] - 0.7 + 0.05
                    z = tcp_data["tcp_pos"]["z"] + 0.1
                    print(f"TCP Data - x: {x}, y: {y}, z: {z}")
                    
                    if time.time() - start_time > 1 / 125:
                        # (-0.12, 0.8 + 0.05, -0.55 + 0.05) -> (-0.12, -0.4, 0.15)
                        # 0.85 -> 0.15?
                        # 
                        
                        
                        target_joint = controller.getInverseKinematics([x, -z, y, 0, 3.14, 3.14])
                        controller.servoJ(target_joint, 0, 0, 0.01, 0.08, 300)
                        start_time = time.time()

            if joint_data_task in done:
                joint_data = await joint_data_task
                if joint_data is not None:
                    # print(f"Joint Data - {joint_data}")
                    pass
            
            tcp_data_task.cancel()
            joint_data_task.cancel()
        except asyncio.CancelledError:
            break
            

async def main(websocket, path):
    print(websocket, path)
    global args, service

    tcp_queue = asyncio.Queue()
    joint_queue = asyncio.Queue()
    stop_event = asyncio.Event()
    
    receive_task = asyncio.create_task(recv_tcp(websocket, tcp_queue, stop_event))
    send_task = asyncio.create_task(send_joint(websocket, joint_queue, stop_event))
    queue_task = asyncio.create_task(process_queues(tcp_queue, joint_queue, stop_event))
    try:
        await asyncio.gather(receive_task, send_task, queue_task, return_exceptions=True)
    except websockets.ConnectionClosed:
        print("WebSocket connection closed in main")
        stop_event.set()
    finally:
        print("Cleaning up tasks in main")
        receive_task.cancel()
        send_task.cancel()
        queue_task.cancel()
        service.disconnect()
        
        await asyncio.gather(receive_task, send_task, queue_task, return_exceptions=True)


if __name__ == "__main__":
    global service
    import argparse

    parser = argparse.ArgumentParser("UR Robot Proxy Server for Data Streaming")
    parser.add_argument("ur_ip", type=str, help="UR Robot IP Address")
    parser.add_argument(
        "--connection_test", action="store_true", help="Connection Test"
    )
    parser.add_argument("--tcp_recv_test", action="store_false", help="TCP Recv Test")
    args = parser.parse_args()

    service = RTDEService(args.ur_ip, 125)
    
    start_server = websockets.serve(main, "0.0.0.0", 5000)
    print("Waiting...")

    loop = asyncio.get_event_loop()
    
    loop.run_until_complete(start_server)
    loop.run_forever()