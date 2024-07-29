from __future__ import print_function
from pymycobot.mycobot import MyCobot
import sys
import termios
import tty
import math
import os

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

msg = """\
Mycobot_280_m5 Teleop Keyboard Controller
---------------------------
Movimg options (control the angle of each joint):
    w: joint2_to_joint1++   s: joint2_to_joint1--
    e: joint3_to_joint2++   d: joint3_to_joint2--
    r: joint4_to_joint3++   f: joint4_to_joint3--
    t: joint5_to_joint4++   g: joint5_to_joint4--
    y: joint6_to_joint5++   h: joint6_to_joint5--
    u: joint6output_to_joint6++ j: joint6output_to_joint6--

Other:
    1 - Go to init pose
    2 - Go to home pose
    3 - Resave home pose
    q - Quit
"""

home_pose = []

class Raw(object):
    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()

    def __enter__(self):
        self.original_stty = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream)

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.stream, termios.TCSANOW, self.original_stty)


class Teleop_Keyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.publisher = self.create_publisher(
            JointTrajectory, "/mycobot280m5_group_controller/joint_trajectory", 10
        )
        self.teleop_keyboard()

    def teleop_keyboard(self):
        data_list = [0, 0, 0, 0, 0, 0]

        try:
            while True:
                try:
                    with Raw(sys.stdin):
                        key = sys.stdin.read(1)
                    if key == "q":
                        break
                    elif key in ["w", "W"]:
                        if data_list[0] < 168:
                            data_list[0] += 1
                    elif key in ["s", "S"]:
                        if data_list[0] > -168:
                            data_list[0] -= 1
                    elif key in ["e", "E"]:
                        if data_list[1] < 135:
                            data_list[1] += 1
                    elif key in ["d", "D"]:
                        if data_list[1] > -135:
                            data_list[1] -= 1
                    elif key in ["r", "R"]:
                        if data_list[2] < 150:
                            data_list[2] += 1
                    elif key in ["f", "F"]:
                        if data_list[2] > -150:
                            data_list[2] -= 1
                    elif key in ["t", "T"]:
                        if data_list[3] < 145:
                            data_list[3] += 1
                    elif key in ["g", "G"]:
                        if data_list[3] > -145:
                            data_list[3] -= 1
                    elif key in ["y", "Y"]:
                        if data_list[4] < 165:
                            data_list[4] += 1
                    elif key in ["h", "H"]:
                        if data_list[4] > -165:
                            data_list[4] -= 1
                    elif key in ["u", "U"]:
                        if data_list[5] < 180:
                            data_list[5] += 1
                    elif key in ["j", "J"]:
                        if data_list[5] > -180:
                            data_list[5] -= 1
                    elif key == "1":
                        data_list = [0, 0, 0, 0, 0, 0]
                    elif key in "2":
                        data_list = home_pose
                    elif key in "3":
                        home_pose = data_list
                    else:
                        continue

                    joint_trajectory = JointTrajectory()
                    joint_trajectory.joint_names = [
                        "joint2_to_joint1",
                        "joint3_to_joint2",
                        "joint4_to_joint3",
                        "joint5_to_joint4",
                        "joint6_to_joint5",
                        "joint6output_to_joint6",
                    ]

                    radian_data_list = [angle * math.pi / 180 for angle in data_list]

                    point = JointTrajectoryPoint()
                    point.positions = radian_data_list
                    point.time_from_start = rclpy.time.Duration(seconds=0.1).to_msg()

                    joint_trajectory.points.append(point)
                    self.publisher.publish(joint_trajectory)
                
                except Exception as e:
                    continue

        except Exception as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)
    teleop = Teleop_Keyboard()

    # print("spin...")
    rclpy.spin(teleop)

    teleop.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    print(msg)
    main()