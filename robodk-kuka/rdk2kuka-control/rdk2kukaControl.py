import robodk.robomath
from robodk.robolink import *
from robodk.robomath import *
import time

linear_speed = 10
angular_speed = 180
joints_speed = 5
joints_accel = 40
is_stop = False


class rdk2kukaControl:
    def __init__(self):
        self.RDK = Robolink()
        self.robot = self.RDK.Item('KUKA KR 70 R2100')
        self.tool = self.robot.Tool()
        self.reference = self.RDK.Item('Baseline')
        self.tcp_obj = self.RDK.Item('tcp_obj')
        self.bbox = self.RDK.Item('bbox')
        self.current_joints = self.robot.SimulatorJoints()

        # bool
        self.is_collide = False
        self.within_bbox = True

        # Boundary limits
        self.min_x, self.max_x, self.min_y, self.max_y, self.min_z, self.max_z = [-90, 90, -90, 90, 10, 190]

        # Initial robot settings
        self.robot.setPoseFrame(self.reference)
        self.robot.setPoseTool(self.tool)
        self.robot.setSpeedJoints(joints_speed)

    def check_bbox(self):
        relative_pose = self.tcp_obj.PoseWrt(self.reference)
        kukaPose = Pose_2_KUKA(relative_pose)
        tcp_x, tcp_y, tcp_z = kukaPose[:3]

        if any([
            tcp_x < self.min_x, tcp_x > self.max_x,
            tcp_y < self.min_y, tcp_y > self.max_y,
            tcp_z < self.min_z, tcp_z > self.max_z
        ]):
            print("TCP out of range!!")
            self.within_bbox = False
            return True
        else:
            print("TCP within range.")
            self.within_bbox = True
            return False

    def collision_detection(self):
        if self.RDK.Collisions() > 0:
            print("Collided!!")
            self.is_collide = True
        else:
            print("is not Collided.")
            self.is_collide = False

    def order2kuka(self):
        start_time = time.time()
        joints = self.robot.SimulatorJoints()
        if self.current_joints != joints:
            print("Change joint values are: ", joints)
            self.robot.MoveJ(joints)
            print("Robot MoveJ")
            self.current_joints = joints

            # Execution time
            print(f"Execution time: {time.time() - start_time} seconds")
            time.sleep(0.004)

    def run(self):
        while True:
            self.check_bbox()
            self.collision_detection()

            if not self.is_collide and self.within_bbox:
                print("order2kuka")
                #self.order2kuka()
            else:
                print("can't order2kuka!!")
                break

        print("rdk2kukaControl Program has ended.")


if __name__ == "__main__":
    kuka_controller = rdk2kukaControl()
    kuka_controller.run()
