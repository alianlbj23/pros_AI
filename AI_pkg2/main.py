import rclpy
from utils.ros_utils import init_ros_node
from modes import rule_base_mode, navigation_mode, robot_arm_mode, destroy_mode
from config import MODES

def main(mode):
    node, ros_thread = init_ros_node()

    mode_functions = {
        "1": rule_base_mode.run,
        "2": navigation_mode.run,
        "3": robot_arm_mode.run,
        "4": destroy_mode.run,
    }

    if mode in mode_functions:
        mode_functions[mode](node)
    else:
        print("Please enter a correct mode number.")

    rclpy.shutdown()
    ros_thread.join()

def print_usage():
    for mode, description in MODES.items():
        print(f" {mode} -- {description}")

if __name__ == "__main__":
    print_usage()
    mode = input("Enter mode: ")
    main(mode)