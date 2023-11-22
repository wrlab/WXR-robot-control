from robodk.robolink import Robolink
import numpy as np
import time

if __name__ == "__main__":
    RDK = Robolink()
    robot = RDK.Item("UR5")
    is_active = False

    joint_data = np.zeros(shape=(0, 6), dtype=np.float32)
    start_time = -1
    fps = 60

    try:
        while True:
            if robot.Busy():
                if (delta := time.time() - start_time) >= 0.01 / fps:
                    print(delta)
                    joint = robot.Joints()
                    joint_data = np.concatenate([joint_data, np.array(joint)])
                    print("Busy", joint)
                    is_active = False
                    start_time = time.time()
            else:
                start_time = time.time()
                if not is_active:
                    print("Stop")
                    is_active = True

    except KeyboardInterrupt:
        print("bye")
        print(joint_data)
        print(len(joint_data))

    np.savetxt("sample-animation.txt", joint_data, fmt="%.12f")
    print("end")
