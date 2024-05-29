import logging
import time, json, math

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

import websockets
import asyncio
import ast


async def _main(websocket, path, host):
    global args
    print(websocket, path)

    reciver = RTDEReceive(host, 125)
    controller = RTDEControl(host)
    step_time_range = 1 / 125

    try:
        # initialize joint position
        if args.tcp_recv_test:
            actual_x, actual_y, actual_z, actual_ax, actual_ay, actual_az = (
                reciver.getActualTCPPose()
            )
            data = await websocket.recv()
            x, y, z, ax, ay, az = ast.literal_eval(data)
            x = round(x, 6)
            y = round(y, 6)
            z = round(z, 6)
            ax = round(ax, 6)
            ay = round(ay, 6)
            az = round(az, 6)

            calc_joints = controller.getInverseKinematics([x, y, z, ax, ay, az])
            if (
                math.sqrt(
                    (x - actual_x) ** 2 + (y - actual_y) ** 2 + (z - actual_z) ** 2
                )
                > 0
            ):
                controller.servoJ(calc_joints, 0, 0, 0.01, 0.03, 300)

            time.sleep(10)
        step_time = time.process_time()

        while True:
            loop_start_time = time.process_time()

            if args.tcp_recv_test:
                data = await websocket.recv()
                x, y, z, ax, ay, az = ast.literal_eval(data)
                x = round(x, 6)
                y = round(y, 6)
                z = round(z, 6)
                ax = round(ax, 6)
                ay = round(ay, 6)
                az = round(az, 6)

                if time.process_time() - step_time > step_time_range:
                    calc_joints = controller.getInverseKinematics([x, y, z, ax, ay, az])
                    controller.servoJ(calc_joints, 0, 0, 0.01, 0.03, 300)
                    step_time = time.process_time()

                print(calc_joints if calc_joints else [0, 0, 0, 0, 0, 0])

            if time.process_time() - step_time > step_time_range:
                if not args.connection_test:
                    joints = reciver.getActualQ()
                    x, y, z, ax, ay, az = reciver.getActualTCPPose()

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

                print(x, y, z)

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
                step_time = time.process_time()

            loop_end_time = time.process_time()
    except Exception as e:
        await websocket.close()
        reciver.disconnect()
        controller.disconnect()
        print("Connection closed")


async def main(websocket, path):
    global args

    _main(websocket, path, args.ur_ip)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser("UR Robot Proxy Server for Data Streaming")
    parser.add_argument("ur_ip", type=str, help="UR Robot IP Address")
    parser.add_argument(
        "--connection_test", action="store_true", help="Connection Test"
    )
    parser.add_argument("--tcp_recv_test", action="store_false", help="TCP Recv Test")
    args = parser.parse_args()

    start_server = websockets.serve(main, "localhost", 5000)
    print("Waiting...")

    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
