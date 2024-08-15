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
            np.deg2rad(180),
            np.deg2rad(70),
            0,
            0,
            0,
        ]
        self.ik_solver = pybulletIK(self.current_angle)
        self.arm_radians_angle = self.current_angle

    def degree_to_radians(self, data):
        return list(np.radians(data))
    def initial_action(self):
        initial_angles = [
            np.deg2rad(90),
            np.deg2rad(30),
            np.deg2rad(160),
            np.deg2rad(180),
            np.deg2rad(110)
        ]
        self.arm_radians_angle = initial_angles  # 更新当前角度为初始动作角度
        self.node.publish_arm(initial_angles)
        # time.sleep(0.5)

    def forward_grap(self):
        new_angles = list(self.arm_radians_angle)
        time.sleep(2)
        if(new_angles[1] > np.deg2rad(120)):
            self.node.publish_arm([-1, -1, -1, -1, np.deg2rad(50)])
            time.sleep(1)
            print("over")
        else:
            # 改延遲太低會造成來不及動作
            new_angles[1] += np.deg2rad(40)
            new_angles[2] -= np.deg2rad(42.5)
            self.arm_radians_angle = new_angles
            self.node.publish_arm(new_angles)
            time.sleep(1)
            self.node.publish_arm([-1, -1, -1, -1, np.deg2rad(50)])

            time.sleep(1)
            initial_angles = [
                np.deg2rad(90),
                np.deg2rad(30),
                np.deg2rad(160),
                np.deg2rad(180),
                np.deg2rad(50)
            ]
            self.arm_radians_angle = initial_angles  # 更新当前角度为初始动作角度
            self.node.publish_arm(initial_angles)

    def put_object(self):
        new_angles = list(self.arm_radians_angle)
        new_angles[1] += np.deg2rad(40)
        new_angles[2] -= np.deg2rad(42.5)
        self.arm_radians_angle = new_angles
        self.node.publish_arm(new_angles)
        time.sleep(1)
        self.node.publish_arm([-1, -1, -1, -1, np.deg2rad(110)])
        time.sleep(1)
        self.initial_action()


    def end_action(self):
        ends_angles = [
            np.deg2rad(90),
            np.deg2rad(30),
            np.deg2rad(160),
            np.deg2rad(180),
            np.deg2rad(50)
        ]
        self.node.publish_arm(ends_angles)
        print("end")
        time.sleep(1)

    def adjust_angles_based_on_direction(self, direction):
        adjustment_step = np.deg2rad(2)  # 每次调整的角度
        adjustment_step2 = np.deg2rad(3)
        new_angles = list(self.arm_radians_angle)

        # if direction == "front":
        #     new_angles[1] += np.deg2rad(3)
        #     new_angles[2] -= np.deg2rad(3)

        if direction == "left":
            new_angles[0] += adjustment_step
        elif direction == "right":
            new_angles[0] -= adjustment_step
        elif direction == "up":
            new_angles[2] -= adjustment_step2
        elif direction == "down":
            new_angles[2] += adjustment_step
            # new_angles[1] += adjustment_step

        elif direction == "front":
            new_angles[1] += np.deg2rad(3)
            new_angles[2] -= np.deg2rad(3)
            pass  # 可以选择在前方不做调整或根据需要进一步处理

        self.arm_radians_angle = new_angles
        self.node.publish_arm(new_angles)
        angles_in_degrees = [np.rad2deg(angle) for angle in new_angles]
        # print(f"Updated Angles (degrees): Joint 1: {angles_in_degrees[0]:.2f}, "
            # f"Joint 2: {angles_in_degrees[1]:.2f}, "
            # f"Joint 3: {angles_in_degrees[2]:.2f}, "
            # f"Joint 4: {angles_in_degrees[3]:.2f}, "
            # f"Joint 5: {angles_in_degrees[4]:.2f}, "
            # f"Joint 6: {angles_in_degrees[5]:.2f}, "
            # f"Joint 7: {angles_in_degrees[6]:.2f}, "
            # f"Joint 8: {angles_in_degrees[7]:.2f}")
        time.sleep(0.5)

    def perform_linear_interpolation(self, target_angles, steps=10):
        # 补齐 target_angles 到与 arm_radians_angle 一致的长度
        if len(target_angles) < len(self.arm_radians_angle):
            target_angles.extend([0] * (len(self.arm_radians_angle) - len(target_angles)))

        # np.linspace 返回 steps 个点，所以索引范围应为 range(steps)
        interpolated_angles_array = np.linspace(self.arm_radians_angle, target_angles, steps)
        fixed_angles = [np.deg2rad(180), np.deg2rad(110)]

        # 计算中间索引位置
        mid_index = steps // 2

        # 获取中间插值角度
        interpolated_angles = interpolated_angles_array[mid_index]
        interpolated_angles[3] = fixed_angles[0]
        interpolated_angles[4] = fixed_angles[1]

        # 发布中间插值角度
        self.node.publish_arm(interpolated_angles)


        # 更新当前角度为目标角度
        self.arm_radians_angle = target_angles
        time.sleep(1)

    # 讓車子距離物體 0.3 以內
    def position_adjustment(self):
        depth = 100
        depth = self.node.get_object_depth()
        direction = self.node.get_object_direction()
        if depth == None:
            depth = 100
        if direction == "left":
            action = "COUNTERCLOCKWISE_ROTATION_SLOW"
            self.node.publish_to_robot(action, pid_control=False)
            time.sleep(0.5)
        elif direction == "right":
            action = "CLOCKWISE_ROTATION_SLOW"
            self.node.publish_to_robot(action, pid_control=False)
            time.sleep(0.5)
        else:
            action = "FORWARD_SLOW"
            self.node.publish_to_robot(action, pid_control=False)
            time.sleep(0.5)

    def precision_grap(self):
        while 1:
            depth = self.node.get_object_depth()
            direction = self.node.get_object_direction()
            if depth < 0.25 and direction == "front":# 往前抓的
                self.forward_grap()
                break
            else:
                self.adjust_angles_based_on_direction(direction)

    def grap(self, tag_name):
        self.initial_action()
        self.node.publish_tag_name("None") # 清空用
        mission_complete = 1
        see_tag_in_moment = 0
        while mission_complete:
            self.node.publish_tag_name(tag_name)
            tag_signal = self.node.get_tag_exist_signal()
            if tag_signal != "0":
                see_tag_in_moment += 1
                data = self.node.get_target_pos()
                # direction = self.node.get_object_direction()
                depth = self.node.get_object_depth()
                if depth == None:
                    depth = 100
                if depth > 0.34:
                    self.position_adjustment()
                else:
                    action = "STOP"
                    self.node.publish_to_robot(action, pid_control=False)
                    time.sleep(2) # 要讓車體穩定
                    self.precision_grap()
                    if tag_signal == "0":
                        mission_complete = 0
                    # self.node.publish_to_robot(action, pid_control=False)
                    #     if depth < 0.25 and direction == "front":# 往前抓的
                    #         self.forward_grap()
                    #         break
                    #     else:
                    #         self.adjust_angles_based_on_direction(direction)
            # 剛剛有瞄到目標
            elif see_tag_in_moment > 2:
                action = "COUNTERCLOCKWISE_ROTATION_SLOW"
                self.node.publish_to_robot(action, pid_control=False)
                see_tag_in_moment = 0
            else:
                action = "COUNTERCLOCKWISE_ROTATION"
                self.node.publish_to_robot(action, pid_control=False)
                time.sleep(0.1)
        action = "STOP"
        self.node.publish_to_robot(action, pid_control=False)
    # def action(self):
    #     print("data receiving")
    #     self.initial_action()
    #     # tag_name = self.node.get_tag_name()
    #     while True:
    #         data = self.node.get_target_pos() # 有時候會出none
    #         direction = self.node.get_object_direction()
    #         depth = self.node.get_object_depth()
    #         if depth == None:
    #             depth = 100
    #             # pass
    #         if depth > 0.35 and depth < 99:
    #              action = "FORWARD_SLOW"
    #              self.node.publish_to_robot(action, pid_control=False)
    #         else:
    #             action = "STOP"
    #             self.node.publish_to_robot(action, pid_control=False)
    #             if depth < 0.25 and direction == "front":# 往前抓的
    #                 print("forward")
    #                 self.forward_grap()
    #                 break
    #             # #     break  # 当达到目标距离时退出循环
    #             # elif depth < 0.3 or data == None:
    #             else:
    #                 self.adjust_angles_based_on_direction(direction)
    #                 # print(direction)

    #             # else:
    #             #     target_coord = data
    #             #     target_coord[2] = -0.2 # 先去wsl的pros_AI看高度參考
    #             #     # 使用逆运动学计算关节角度
    #             #     radians = self.ik_solver.pybullet_move(target_coord, self.current_angle)
    #             #     radians = list(radians)
    #             #     radians[3] = np.deg2rad(180)
    #             #     radians[4] = np.deg2rad(110)
    #             #     radians = radians[0:5]
    #                 # self.perform_linear_interpolation(radians, steps=10)
    #             print("no 2 :" + str(depth))

# def main(args=None): #
#     rclpy.init(args=args)
#     node = rclpy.create_node("robot_arm_control_node")
#     robot_arm_control = RobotArmControl(node)

#     try:
#         robot_arm_control.action()
#     except KeyboardInterrupt:
#         robot_arm_control.end_action()
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()
