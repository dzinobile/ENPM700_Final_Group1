# my_package/global_map_tf_publisher.py

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class GlobalMapTFPublisher(Node):
    def __init__(self):
        super().__init__("global_map_tf_publisher")
        self.broadcaster = StaticTransformBroadcaster(self)

        transforms = []

        # world -> robot1/map
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = "world"
        t1.child_frame_id = "robot1/map"
        t1.transform.translation.x = 0.5
        t1.transform.translation.y = 0.05
        t1.transform.translation.z = 0.05
        t1.transform.rotation.w = 1.0
        transforms.append(t1)

        # world -> robot2/map
        t2 = TransformStamped()
        t2.header.stamp = t1.header.stamp
        t2.header.frame_id = "world"
        t2.child_frame_id = "robot2/map"
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.05
        t2.transform.translation.z = 0.05
        t2.transform.rotation.w = 1.0
        transforms.append(t2)

        self.broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalMapTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

