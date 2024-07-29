import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math


class Slider_Subscriber(Node):
    def __init__(self):
        super().__init__("control_slider")
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.listener_callback,
            10
        )
        self.subscription

        self.publisher = self.create_publisher(
            JointTrajectory,
            "/mycobot280m5_group_controller/joint_trajectory",
            10
        )

    def listener_callback(self, msg):
        joint_trajectory = JointTrajectory()
        joint_trajectory.header.frame_id = "base_link"
        joint_trajectory.joint_names = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ]

        point = JointTrajectoryPoint()
        point.positions = msg.position
        point.time_from_start = rclpy.time.Duration(seconds=1).to_msg()

        joint_trajectory.points.append(point)

        # data_list = []
        # for _, value in enumerate(msg.position):
        #     radians_to_angles = round(math.degrees(value), 2)
        #     data_list.append(radians_to_angles)

        # print('data_list: {}'.format(data_list))
        self.publisher.publish(joint_trajectory)


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()

    print("spin...")
    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()