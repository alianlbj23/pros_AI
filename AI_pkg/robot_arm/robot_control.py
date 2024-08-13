import rclpy
import math
import numpy as np
import time
from robot_arm.ik2 import pybulletIK
import cv2

class RobotArmControl:
    def __init__(self, node):
        self.node = node
        self.current_angle = [
            np.deg2rad(90),
            np.deg2rad(30),
            np.deg2rad(160),
            np.deg2rad(50),
            np.deg2rad(70),
            0,
            0,
            0,
        ]
        self.ik_solver = pybulletIK(self.current_angle)
        self.arm_radians_angle = self.current_angle

    def degree_to_radians(self, data):
        return list(np.radians(data))

    def end_action(self):
        self.node.publish_arm([-1, -1, -1, -1, np.deg2rad(0)])

    def adjust_angles_based_on_direction(self, direction):
        adjustment_step = np.deg2rad(5)  # 每次调整的角度
        new_angles = list(self.arm_radians_angle)

        # if direction == "front":
        #     new_angles[1] += adjustment_step
        #     new_angles[2] -= adjustment_step

        if direction == "left":
            new_angles[0] += adjustment_step
        elif direction == "right":
            new_angles[0] -= adjustment_step
        elif direction == "up":
            new_angles[2] -= adjustment_step
        elif direction == "down":
            # new_angles[2] += adjustment_step
            new_angles[1] += adjustment_step

        elif direction == "front":
            new_angles[1] += adjustment_step
            new_angles[2] -= adjustment_step
        #     pass  # 可以选择在前方不做调整或根据需要进一步处理

        self.arm_radians_angle = new_angles
        self.node.publish_arm(new_angles)
        angles_in_degrees = [np.rad2deg(angle) for angle in new_angles]
        print(f"Updated Angles (degrees): Joint 1: {angles_in_degrees[0]:.2f}, "
            f"Joint 2: {angles_in_degrees[1]:.2f}, "
            f"Joint 3: {angles_in_degrees[2]:.2f}, "
            f"Joint 4: {angles_in_degrees[3]:.2f}, "
            f"Joint 5: {angles_in_degrees[4]:.2f}, "
            f"Joint 6: {angles_in_degrees[5]:.2f}, "
            f"Joint 7: {angles_in_degrees[6]:.2f}, "
            f"Joint 8: {angles_in_degrees[7]:.2f}")
        time.sleep(0.1)

    def perform_linear_interpolation(self, target_angles, steps=10):
        # 补齐 target_angles 到与 arm_radians_angle 一致的长度
        if len(target_angles) < len(self.arm_radians_angle):
            target_angles.extend([0] * (len(self.arm_radians_angle) - len(target_angles)))

        # np.linspace 返回 steps 个点，所以索引范围应为 range(steps)
        interpolated_angles_array = np.linspace(self.arm_radians_angle, target_angles, steps)
        fixed_angles = [np.deg2rad(50), np.deg2rad(70)]
        for i in range(steps):
            interpolated_angles = interpolated_angles_array[i]
            interpolated_angles[3] = fixed_angles[0]
            interpolated_angles[4] = fixed_angles[1]
            self.node.publish_arm(interpolated_angles)
            time.sleep(0.5)  # 等待机械臂执行动作

        self.arm_radians_angle = target_angles  # 更新当前角度为目标角度



    def action(self):
        print("data receiving")
        while True:
            data = None
            while data is None:
                data = self.node.get_target_pos()
            direction = self.node.get_object_direction()
            self.adjust_angles_based_on_direction(direction)
            # print(data[0])

            # if data[0] < 0.2:
            #     self.end_action()
            #     break  # 当达到目标距离时退出循环
            # elif data[0] < 0.3:
            #     direction = self.node.get_object_direction()
            #     self.adjust_angles_based_on_direction(direction)
            # else:
            #     target_coord = data
            #     # 使用逆运动学计算关节角度
            #     radians = self.ik_solver.pybullet_move(target_coord, self.current_angle)
            #     radians = list(radians)
            #     radians[3] = np.deg2rad(50)
            #     radians[4] = np.deg2rad(70)
            #     radians = radians[0:5]

            #     self.perform_linear_interpolation(radians, steps=10)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("robot_arm_control_node")
    robot_arm_control = RobotArmControl(node)

    try:
        robot_arm_control.action()
    except KeyboardInterrupt:
        robot_arm_control.end_action()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
