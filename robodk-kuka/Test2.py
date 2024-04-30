from robodk.robolink import *
from robodk.robomath import *
import time
import asyncio


if __name__ == "__main__":
    RDK = Robolink()
    robot = RDK.Item('KUKA KR 70 R2100-Meltio')
    tool = robot.Tool()
    reference = RDK.Item('Baseline')
    tcp_obj = RDK.Item('tcp_obj')
    bbox = RDK.Item('bbox')

    print("current RunMode(): ", RDK.RunMode())
    current_joints = robot.SimulatorJoints()
    print("current Sim Joints: ", robot.SimulatorJoints())

    RDK.setRunMode(RUNMODE_RUN_ROBOT)
    print("current Sim Joints: ", robot.SimulatorJoints())
    print("current RunMode(): ", RDK.RunMode())
    #joints = [0.3669306773580602, -84.24947731080498, 112.5832912167649, 4.3010636150456225, -28.56989649101972, -3.7847460949017755]
    robot.MoveJ(robot.SimulatorJoints())
