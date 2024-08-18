import rclpy
from math import pi
from utils.rotate_angle import calculate_angle_point, calculate_angle_to_target
import csv
from car_navigation.navigation_process import NavigationProcess


class NavigationController:
    def __init__(self, node):
        self.NavigationProcess = NavigationProcess(node=node)
        self.node = node
    def run(self):
        while rclpy.ok():
            self.NavigationProcess.run()

    def nav_to_target(self, target_position):
        self.node.publish_goal_pose(target_position)
        self.NavigationProcess.run()
