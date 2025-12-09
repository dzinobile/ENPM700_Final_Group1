import math

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile


# Robot geometry (match your Webots world)
HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025


class MyRobotDriver:
    def init(self, webots_node, properties):
        # Webots handle
        self.__robot = webots_node.robot
        self.__robot_name = self.__robot.getName()  # "robot1", "robot2", "robot3"

        # Simulation timestep
        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.__dt = self.__timestep / 1000.0

        # Motors
        self.__left_motor = self.__robot.getDevice("left wheel motor")
        self.__right_motor = self.__robot.getDevice("right wheel motor")

        self.__left_motor.setPosition(float("inf"))
        self.__left_motor.setVelocity(0.0)

        self.__right_motor.setPosition(float("inf"))
        self.__right_motor.setVelocity(0.0)

        # Lidar device (must exist in world as Lidar { name "lidar" })
        try:
            self.__lidar = self.__robot.getDevice("lidar")
            self.__lidar.enable(self.__timestep)
        except Exception:
            self.__lidar = None

        # Commanded twist
        self.__target_twist = Twist()

        # Integrated odom state (dead-reckoning)
        self.__x = 0.0
        self.__y = 0.0
        self.__theta = 0.0

        # ROS node
        rclpy.init(args=None)
        self.__node = rclpy.create_node(f"{self.__robot_name}_driver")

        qos = QoSProfile(depth=10)

        # Subscribe to per-robot cmd_vel
        cmd_vel_topic = f"/{self.__robot_name}/cmd_vel"
        self.__node.get_logger().info(f"Subscribing to {cmd_vel_topic}")
        self.__node.create_subscription(
            Twist, cmd_vel_topic, self.__cmd_vel_callback, qos
        )

        # Publishers: odom and tf
        self.__odom_pub = self.__node.create_publisher(
            Odometry, f"/{self.__robot_name}/odom", qos
        )
        self.__tf_broadcaster = TransformBroadcaster(self.__node)

        # Optional LaserScan publisher
        if self.__lidar is not None:
            self.__scan_pub = self.__node.create_publisher(
                LaserScan, f"/{self.__robot_name}/scan", qos
            )
        else:
            self.__scan_pub = None
            self.__node.get_logger().warn(
                "No lidar device named 'lidar' found. LaserScan will not be published."
            )

    def __cmd_vel_callback(self, twist: Twist):
        # Store last commanded velocity
        self.__target_twist = twist

    def step(self):
        # Pump ROS callbacks
        rclpy.spin_once(self.__node, timeout_sec=0.0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        # Differential drive inverse kinematics
        command_motor_left = (
            forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS
        ) / WHEEL_RADIUS
        command_motor_right = (
            forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS
        ) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)

        # Integrate simple odometry in the plane
        self.__theta += angular_speed * self.__dt
        # Normalize yaw
        self.__theta = math.atan2(math.sin(self.__theta), math.cos(self.__theta))

        self.__x += forward_speed * math.cos(self.__theta) * self.__dt
        self.__y += forward_speed * math.sin(self.__theta) * self.__dt

        now = self.__node.get_clock().now().to_msg()

        # Publish nav_msgs/Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = f"odom_{self.__robot_name}"
        odom.child_frame_id = f"{self.__robot_name}/base_link"
        odom.pose.pose.position.x = self.__x
        odom.pose.pose.position.y = self.__y
        odom.pose.pose.position.z = 0.0

        # Yaw to quaternion (z-w only for planar motion)
        qz = math.sin(self.__theta / 2.0)
        qw = math.cos(self.__theta / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = forward_speed
        odom.twist.twist.angular.z = angular_speed

        self.__odom_pub.publish(odom)

        # Publish TF: odom_<robot> -> <robot>/base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = odom.header.frame_id
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = self.__x
        t.transform.translation.y = self.__y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.__tf_broadcaster.sendTransform(t)

        # Publish LaserScan if lidar exists
        if self.__lidar is not None and self.__scan_pub is not None:
            self._publish_scan(now)

    def _publish_scan(self, stamp):
        # Webots Lidar: range image and intrinsic params
        horizontal_res = self.__lidar.getHorizontalResolution()
        fov = self.__lidar.getFov()
        max_range = self.__lidar.getMaxRange()
        min_range = 0.01  # small positive range

        ranges = list(self.__lidar.getRangeImage())

        scan = LaserScan()
        scan.header.stamp = stamp
        # For now, use base_link as the scan frame (no extra TF needed)
        scan.header.frame_id = f"{self.__robot_name}/base_link"

        scan.angle_min = -fov / 2.0
        scan.angle_max = fov / 2.0
        scan.angle_increment = fov / float(horizontal_res)
        scan.range_min = min_range
        scan.range_max = max_range
        scan.scan_time = self.__dt
        scan.time_increment = 0.0

        # Clamp and sanitize ranges
        scan.ranges = [
            max(min(r, max_range), min_range) if math.isfinite(r) else max_range
            for r in ranges
        ]

        self.__scan_pub.publish(scan)
