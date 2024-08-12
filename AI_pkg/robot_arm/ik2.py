import pybullet as p
import pybullet_data
import numpy as np
import os
import threading
import time


class pybulletIK:
    def __init__(self, first_angle):
        self.angle = first_angle
        # 计算基座位置的偏移量
        # base_position_offset = (
        #     -2.989375374043843,
        #     -1.1164329046590977,
        #     -0.031849440895134605,
        # )

        # 初始化 PyBullet 仿真环境
        p.connect(p.GUI)  # 使用 GUI 模式，这样你可以看到仿真界面
        p.setAdditionalSearchPath(
            pybullet_data.getDataPath()
        )  # 设置搜索路径以找到 URDF 文件
        p.loadURDF("plane.urdf")  # 加载平面

        current_directory = os.path.dirname(os.path.abspath(__file__))
        urdf_file_path = os.path.join(
            current_directory, "arm_ver6", "test_abb_4600.urdf"
        )

        # 使用计算出的偏移量来加载机器人
        self.robot_id = p.loadURDF(
            urdf_file_path,
            basePosition=(0, 0, 0),
            baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi / 2]),
            useFixedBase=True,
        )
        self.num_joints = p.getNumJoints(self.robot_id)

        # 关闭手臂的物理碰撞
        self.disable_collision_for_robot()

        # 设置初始关节角度
        self.set_joint_angles(first_angle)

        # 初始化相机（固定在第1轴的前方1公分）
        self.camera_link_index = 0  # 假设第1轴的索引为0
        self.camera_position = [0, 0, 0]  # 相机在第1轴正前方2公分
        self.camera_orientation = [0, 0, 0, 1]  # 保持相机方向不变（四元数）

        # 将红色方块作为深度相机的表示
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_BOX, rgbaColor=[1, 0, 0, 1], halfExtents=[0.02, 0.02, 0.02]
        )
        self.camera_marker = p.createMultiBody(
            baseVisualShapeIndex=visual_shape_id, basePosition=self.camera_position
        )
        p.setCollisionFilterPair(
            self.robot_id, self.camera_marker, -1, -1, 0
        )  # 停止碰撞

        # 初始化目标物体（绿色方块）
        self.target_marker = self.create_target_marker()

        # 创建并启动仿真线程
        self.update_camera_marker()
        self.simulation_thread = threading.Thread(target=self.run_simulation)
        self.simulation_thread.start()

    def disable_collision_for_robot(self):
        """关闭机械手臂的物理碰撞"""
        for i in range(self.num_joints):
            p.setCollisionFilterGroupMask(
                self.robot_id, i, collisionFilterGroup=0, collisionFilterMask=0
            )

    def set_joint_angles(self, joint_angles):
        """应用初始关节角度到机器人"""
        for i in range(self.num_joints):
            p.resetJointState(self.robot_id, i, joint_angles[i])

    def create_target_marker(self):
        """创建绿色方块表示目标物体"""
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_BOX, rgbaColor=[0, 1, 0, 1], halfExtents=[0.02, 0.02, 0.02]
        )
        target_marker = p.createMultiBody(
            baseVisualShapeIndex=visual_shape_id, basePosition=[0, 0, 0]
        )
        p.setCollisionFilterPair(self.robot_id, target_marker, -1, -1, 0)  # 停止碰撞
        return target_marker

    def print_base_position(self):
        """打印机械手臂基座的世界坐标"""
        base_position, base_orientation = p.getBasePositionAndOrientation(self.robot_id)
        print(f"Base position: {base_position}")

    def run_simulation(self):
        """运行仿真"""
        while True:
            p.stepSimulation()
            # 更新相机标记位置
            time.sleep(1 / 240)
            p.setTimeStep(1 / 240)
            p.setGravity(0, 0, -9.81)

    def update_camera_marker(self):
        """更新深度相机红色方块的位置"""
        link_state = p.getLinkState(self.robot_id, self.camera_link_index)
        link_world_position = link_state[0]
        link_world_orientation = link_state[1]
        p.resetBasePositionAndOrientation(
            self.camera_marker, link_world_position, link_world_orientation
        )

    def update_target_marker(self, target_position):
        """更新目标物体（绿色方块）的世界坐标"""
        p.resetBasePositionAndOrientation(
            self.target_marker, target_position, [0, 0, 0, 1]
        )

    def transform_camera_to_base(self, camera_position):
        """将相机坐标系中的位置转换为基座坐标系"""
        # 获取相机在世界坐标系中的位置和方向
        link_state = p.getLinkState(self.robot_id, self.camera_link_index)
        camera_world_position = np.array(link_state[0])
        camera_world_orientation = link_state[1]

        # 将相机的四元数转换为旋转矩阵
        rotation_matrix = np.array(
            p.getMatrixFromQuaternion(camera_world_orientation)
        ).reshape(3, 3)

        # 将物体的相机坐标系位置转换到世界坐标系
        world_position = (
            np.dot(rotation_matrix, camera_position) + camera_world_position
        )

        # 获取基座的世界坐标系位置和方向
        base_position, base_orientation = p.getBasePositionAndOrientation(self.robot_id)
        base_position = np.array(base_position)
        base_rotation_matrix = np.array(
            p.getMatrixFromQuaternion(base_orientation)
        ).reshape(3, 3)

        # 将世界坐标系位置转换为基座坐标系位置
        relative_position = np.dot(
            base_rotation_matrix.T, world_position - base_position
        )

        return relative_position

    def pybullet_move(self, target_camera_coords, current_joint_angles):
        # 确保目标在相机的后方并在机械手臂的可达范围内
        base_link_state = p.getLinkState(self.robot_id, self.camera_link_index)
        base_link_position = np.array(base_link_state[0])  # 位置
        base_link_orientation = base_link_state[1]  # 方向
        rotation_matrix = np.array(
            p.getMatrixFromQuaternion(base_link_orientation)
        ).reshape(3, 3)
        target_camera_coords = np.array(target_camera_coords)
        # target_world_coords = (
        #     np.dot(rotation_matrix, target_camera_coords) + base_link_position
        # )
        target_world_coords = target_camera_coords + base_link_position
        target_camera_coords[0] = -abs(target_camera_coords[0])  # 反转 X 轴
        # target_camera_coords[0] = -abs(target_camera_coords[0])
        # target_camera_coords[2] = -abs(target_camera_coords[2])  # 反转 Y 轴

        # 将目标从相机坐标系转换到基座坐标系
        # target_base_coords = self.transform_camera_to_base(target_camera_coords)
        target_base_coords = target_world_coords
        # tmp = target_base_coords[0]
        # target_base_coords[0] = target_base_coords[1]
        # target_base_coords[1] = tmp
        # target_base_coords[2] = 0.1
        # print(target_base_coords)

        # 更新目标物体位置
        self.update_target_marker(target_base_coords)

        # 执行逆运动学计算
        joint_angles = p.calculateInverseKinematics(
            self.robot_id,
            6,  # 末端执行器的链接索引
            target_base_coords,
            lowerLimits=[-np.pi] * self.num_joints,
            upperLimits=[np.pi] * self.num_joints,
            jointRanges=[2 * np.pi] * self.num_joints,
            restPoses=current_joint_angles,
        )

        # 提取关节角度
        joint_angles = joint_angles[: self.num_joints]

        # 将关节角度应用到机器人模型
        for i in range(self.num_joints):
            p.setJointMotorControl2(
                self.robot_id, i, p.POSITION_CONTROL, joint_angles[i]
            )

        return joint_angles


def main():
    # 在此处设置你想要的初始关节角度
    initial_joint_angles = [
        np.deg2rad(90),  # 第一轴
        np.deg2rad(0),  # 第二轴
        np.deg2rad(160),  # 第三轴
        np.deg2rad(0),  # 第四轴
        np.deg2rad(0),  # 第五轴
        np.deg2rad(0),  # 第六轴
        np.deg2rad(0),  # 第七轴
        np.deg2rad(0),  # 第八轴
    ]

    ik_solver = pybulletIK(initial_joint_angles)

    while True:
        user_input = input("Enter target coordinates relative to camera (x y z): ")
        target_camera_coords = [float(coord) for coord in user_input.split()]
        print(f"Target position relative to camera: {target_camera_coords}")

        current_joint_angles = [0, 0, 0, 0, 0, 0, 0, 0]  # 根据实际情况设置初始关节角度
        ik_solver.pybullet_move(target_camera_coords, current_joint_angles)
        # 每次移动后打印基座位置
        ik_solver.print_base_position()


if __name__ == "__main__":
    main()
