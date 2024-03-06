from robodk.robolink import *
import asyncio


async def send2kuka(robot, stop_event):
    while not stop_event.is_set():
        # 로봇 제어 로직
        joints = robot.Joints()
        print("Current joint values are:", joints)

        robot.MoveJ(joints)
        print("Robot MoveJ")

        await asyncio.sleep(1)
