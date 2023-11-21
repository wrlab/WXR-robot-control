from robodk.robolink import Robolink
from config import Config
import socket
import websockets
import asyncio


if __name__ == "__main__":
    RDK = Robolink()
    robot = RDK.Item("UR5")

    is_active = False
    active_count = 0

    async def handler(websocket):
        global is_active
        while True:
            if robot.Busy():
                data = str(robot.Joints().list())
                formatted_data = f"{len(data):05}|{data}"
                print("Busy", formatted_data)
                is_active = False
                await websocket.send(formatted_data)
            else:
                if not is_active:
                    print("Stop")
                    is_active = True

    server = websockets.serve(handler, Config.HOST, Config.PORT)
    asyncio.get_event_loop().run_until_complete(server)
    asyncio.get_event_loop().run_forever()
