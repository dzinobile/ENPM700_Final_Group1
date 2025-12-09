import math

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSProfile


# Robot geometry, match your Webots model
HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025


class MyRobotDriver:
    def init(self, webots_node, properties):
        # Webots handle
        self.__robot = webots_node.robot
        self.__robot_name = self.__robot.getName()  # "robot1", "robot2", ...

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

        # Lidar frame id as used by Webots plugin topics
        # Webots publishes LaserScan on /robotX/lidar1 with frame_id "robotX/lidar1"
        self.__lidar_frame_id = f"{self.__robot_name}/lidar1"
        print(self.__lidar_frame_id)
        print("===========================")
        # self.__lidar = None
        # self.__lidar = self.__robot.getDevice("lidar1")
        # self.__lidar.enable(self.__timestep)
        # self.__lidar_frame_id = f"{self.__robot_name}/{self.__lidar.getName()}"

        # Commanded twist
        self.__target_twist = Twist()

        # Dead reckoning odom state
        self.__x = 0.0
        self.__y = 0.0
        self.__theta = 0.0

        # ROS node
        rclpy.init(args=None)
        self.__node = rclpy.create_node(f"{self.__robot_name}_driver")

        qos = QoSProfile(depth=10)

        # Subscribe to cmd_vel
        cmd_vel_topic = f"/{self.__robot_name}/cmd_vel"
        self.__node.get_logger().info(f"Subscribing to {cmd_vel_topic}")
        self.__node.create_subscription(
            Twist, cmd_vel_topic, self.__cmd_vel_callback, qos
        )

        # Publishers: odom and TF
        self.__odom_pub = self.__node.create_publisher(
            Odometry, f"/{self.__robot_name}/odom", qos
        )
        self.__tf_broadcaster = TransformBroadcaster(self.__node)

        # Static TF broadcaster for base_link -> lidar
        self.__static_tf_broadcaster = StaticTransformBroadcaster(self.__node)
        self._publish_static_lidar_tf()

    def __cmd_vel_callback(self, twist: Twist):
        self.__target_twist = twist

    def _publish_static_lidar_tf(self):
        """
        Publish a static transform from base_link to the lidar frame
        for this robot. Right now we assume the lidar is at the base origin.
        Later you can plug the actual offset from the Webots model.
        """
        now = self.__node.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = f"{self.__robot_name}/base_link"
        t.child_frame_id = self.__lidar_frame_id

        # If your lidar is offset in Webots, plug those values here instead.
        # For now treat it as coincident with base_link.
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1

        # No rotation between base_link and lidar frame yet
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.__static_tf_broadcaster.sendTransform(t)

        self.__node.get_logger().info(
            f"Published static TF {t.header.frame_id} -> {t.child_frame_id}"
        )

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

        # Integrate odom
        self.__theta += angular_speed * self.__dt
        self.__theta = math.atan2(math.sin(self.__theta), math.cos(self.__theta))

        self.__x += forward_speed * math.cos(self.__theta) * self.__dt
        self.__y += forward_speed * math.sin(self.__theta) * self.__dt

        now = self.__node.get_clock().now().to_msg()

        # Odom message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = f"odom_{self.__robot_name}"
        odom.child_frame_id = f"{self.__robot_name}/base_link"

        odom.pose.pose.position.x = self.__x
        odom.pose.pose.position.y = self.__y
        odom.pose.pose.position.z = 0.0

        qz = math.sin(self.__theta / 2.0)
        qw = math.cos(self.__theta / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = forward_speed
        odom.twist.twist.angular.z = angular_speed

        self.__odom_pub.publish(odom)

        # Dynamic TF odom -> base_link
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
