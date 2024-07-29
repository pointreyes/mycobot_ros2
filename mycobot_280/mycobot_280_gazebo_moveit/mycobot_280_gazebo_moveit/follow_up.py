import rclpy
from pymycobot.mycobot import MyCobot
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import os
import math


class Talker(Node):
    def __init__(self):
        super().__init__("follow_display")
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
   
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        
        self.robot_m5 = os.popen("ls /dev/ttyUSB*").readline()[:-1]
        self.robot_wio = os.popen("ls /dev/ttyACM*").readline()[:-1]
        if self.robot_m5:
            port = self.robot_m5
        else:
            port = self.robot_wio

        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.mc = MyCobot(port, str(baud))
        self.mc.release_all_servos()

    def start(self):
        pub = self.create_publisher(
            msg_type=JointTrajectory,
            topic="/mycobot280m5_group_controller/joint_trajectory",
            qos_profile=10
        )
        rate = self.create_rate(30)

        while rclpy.ok():
            rclpy.spin_once(self)
            try:       
                joint_trajectory = JointTrajectory()
                joint_trajectory.header = Header()
                joint_trajectory.joint_names = [
                    "joint2_to_joint1",
                    "joint3_to_joint2",
                    "joint4_to_joint3",
                    "joint5_to_joint4",
                    "joint6_to_joint5",
                    "joint6output_to_joint6",
                ]
                angles = self.mc.get_radians()
                data_list = []
                for _, value in enumerate(angles):
                    data_list.append(value)

                point = JointTrajectoryPoint()
                # self.get_logger().info('angles: {}'.format([round(math.degrees(angle), 2) for angle in data_list]))
                point.positions = data_list
                point.time_from_start = rclpy.time.Duration(seconds=1).to_msg()

                joint_trajectory.points.append(point)
                pub.publish(joint_trajectory)

                rate.sleep()
            except Exception as e:
                print(e)

        
def main(args=None):
    rclpy.init(args=args)
    
    talker = Talker()
    talker.start()
    rclpy.spin(talker)
    
    talker.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()