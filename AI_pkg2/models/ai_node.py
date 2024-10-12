import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np

class AI_node(Node):
    def __init__(self):
        super().__init__('ai_node')
        self.get_logger().info('Initializing AI Node')

        # QoS configuration
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize subscribers
        self.init_subscribers()

        # Initialize publishers
        self.init_publishers()

        # Initialize TF components
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize data storage
        self.laser_data = None
        self.odom_data = None
        self.current_pose = None

        self.get_logger().info('AI Node initialized successfully')

    def init_subscribers(self):
        # Subscribe to LaserScan data
        self.create_subscription(LaserScan, '/scan', self.laser_callback, self.qos_profile)
        
        # Subscribe to Odometry data
        self.create_subscription(Odometry, '/odom', self.odom_callback, self.qos_profile)
        
        # Subscribe to car position
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.subscribe_callback_amcl, 10
        )

        # Subscribe to goal position
        self.create_subscription(
            PoseStamped, "/goal_pose", self.subscribe_callback_goal, 1
        )

        # Subscribe to navigation command
        self.create_subscription(
            Twist, "/cmd_vel_nav", self.navigation_callback, 1
        )
        






        # Add any additional subscriptions here
        # Example: self.create_subscription(SomeMsgType, 'some_topic', self.some_callback, self.qos_profile)

    def init_publishers(self):
        self.processed_laser_pub = self.create_publisher(String, 'processed_laser', 10)
        self.processed_odom_pub = self.create_publisher(PoseStamped, 'processed_odom', 10)

    def laser_callback(self, msg):
        self.laser_data = msg
        processed_data = self.process_laser_data(msg)
        self.publish_processed_laser(processed_data)

    def odom_callback(self, msg):
        self.odom_data = msg
        self.current_pose = self.process_odom_data(msg)
        self.publish_processed_odom(self.current_pose)

    def process_laser_data(self, laser_msg):
        # Process laser data, e.g., find the nearest obstacle
        ranges = np.array(laser_msg.ranges)
        min_distance = np.min(ranges)
        min_angle = laser_msg.angle_min + np.argmin(ranges) * laser_msg.angle_increment
        return f"Nearest obstacle: distance={min_distance:.2f}, angle={min_angle:.2f}"

    def process_odom_data(self, odom_msg):
        # Process odometry data
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose = odom_msg.pose.pose
        return pose

    def publish_processed_laser(self, data):
        msg = String()
        msg.data = data
        print("lidar : ", msg.data)
        self.processed_laser_pub.publish(msg)

    def publish_processed_odom(self, pose):
        self.processed_odom_pub.publish(pose)

    def get_current_pose(self):
        return self.current_pose

    def get_laser_data(self):
        return self.laser_data

def main(args=None):
    rclpy.init(args=args)
    node = AI_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down AI Node')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()