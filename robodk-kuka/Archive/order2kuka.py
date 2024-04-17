from robodk.robolink import *
import asyncio


async def send2kuka(robot, stop_event):
    while not stop_event.is_set():
        target_joints = [0, -90, 90, 0, 0, 0]
        print(robot)
        # 로봇 제어 로직
        joints = robot.Joints()
        print("Current joint values are:", joints)

        #robot.MoveJ(joints)
        robot.MoveJ(target_joints)
        print("Robot MoveJ")

        await asyncio.sleep(1)
