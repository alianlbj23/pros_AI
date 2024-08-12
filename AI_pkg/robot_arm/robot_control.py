import rclpy
import math
import numpy as np
import time
from robot_arm.ik2 import pybulletIK
import cv2

# import roslibpy  # 导入 roslibpy 库
from ros_receive_and_data_processing.car_models import *


class RobotArmControl:
    def __init__(self, node):
        self.node = node
        self.current_angle = [
            np.deg2rad(90),
            np.deg2rad(0),
            np.deg2rad(160),
            0,
            0,
            0,
            0,
            0,
        ]
        self.ik_solver = pybulletIK(self.current_angle)

        # 连接到 ROSBridge
        self.rosbridge_client = roslibpy.Ros(host="192.168.0.207", port=9090)
        self.rosbridge_client.run()

        self.roslib_publisher = roslibpy.Topic(
            self.rosbridge_client,
            DeviceDataTypeEnum.robot_arm,
            "trajectory_msgs/JointTrajectoryPoint",
        )

    def degree_to_radians(self, data):
        return list(np.radians(data))

    def end_action(self):
        end_action_angle = self.degree_to_radians([90, 0, 160, 50, 10])
        self.node.publish_arm(end_action_angle)

    def action(self):
        print("data receiving")
        while True:
            data = None
            while data == None:
                data = self.node.get_target_pos()

            # 获取目标坐标
            target_coord = data

            # 使用逆运动学计算关节角度
            radians = self.ik_solver.pybullet_move(target_coord, self.current_angle)

            # 发布关节角度到机械臂
            self.node.publish_arm(radians)

            # 创建 JointTrajectoryPoint 消息
            # message = roslibpy.Message(
            #     {
            #         "positions": radians,
            #         "velocities": [],  # 可以为空，或者填入速度值
            #         "accelerations": [],  # 可以为空，或者填入加速度值
            #         "effort": [],  # 可以为空，或者填入力矩值
            #         "time_from_start": {"secs": 0, "nsecs": 0},  # 可以根据需要设置
            #     }
            # )

            # 发布 JointTrajectoryPoint 消息到 ROSBridge
            # self.roslib_publisher.publish(message)

            # time.sleep(1)
            # self.current_angle = radians  # 更新当前角度


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
