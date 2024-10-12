import rclpy
import threading
from models.ai_node import AI_node

def init_ros_node():
    """Initialize ROS node and start a thread to run it."""
    rclpy.init()
    node = AI_node()
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()
    return node, thread