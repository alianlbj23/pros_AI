from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    PoseStamped,
    Twist,
    TransformStamped,
    Point,
)
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData, Odometry
import random
from std_msgs.msg import Header, Float32, String
import json
import orjson
import math
import time
from rclpy.node import Node
from ros_receive_and_data_processing.car_models import *
from ros_receive_and_data_processing.data_transform import preprocess_data
from ros_receive_and_data_processing.config import (
    ACTION_MAPPINGS,
    LIDAR_PER_SECTOR,
    NEXT_POINT_DISTANCE,
    FRONT_LIDAR_INDICES,
    LEFT_LIDAR_INDICES,
    RIGHT_LIDAR_INDICES,
)
from trajectory_msgs.msg import JointTrajectoryPoint
from PIL import Image
import yaml
import os
from tf2_ros import StaticTransformBroadcaster


class AI_node(Node):
    def __init__(self):
        super().__init__("aiNode")
        self.get_logger().info("Ai start")
        self.real_car_data = {}
        self.data_to_AI = ""
        self.lastest_data = None

        """
        以下字典是用來定義這個node接收到最新的數值後傳送到UnityAdaptor.py內轉換
        最終轉換格式是UnityAdaptor.py決定 這邊只負責接收最新數值
        """
        self.real_car_data["ROS2CarPosition"] = []
        self.real_car_data["ROS2CarQuaternion"] = []
        self.real_car_data["ROS2TargetPosition"] = []

        #  讀取四輪電壓
        self.real_car_data["ROS2WheelAngularVelocityLeftBack"] = 0.01
        self.real_car_data["ROS2WheelAngularVelocityLeftFront"] = 0.01
        self.real_car_data["ROS2WheelAngularVelocityRightBack"] = 0.01
        self.real_car_data["ROS2WheelAngularVelocityRightFront"] = 0.01

        #  lidar的射線
        self.real_car_data["ROS2Range"] = []
        # lidar每條射線的vector
        self.real_car_data["ROS2RangePosition"] = []

        #  navigation資料
        self.real_car_data["cmd_vel_nav"] = Twist()
        self.real_car_data["received_global_plan"] = None

        # 目標座標
        self.real_car_data["arm_tartget_position"] = None
        # 物體在相機的哪個方向
        self.real_car_data["object_direction"] = None
        self.real_car_data["object_depth"] = None
        self.real_car_data["tag_exist"] = "0"
        self.real_car_data["arucode_depth"] = 0
        self.real_car_data["arucode_direction"] = 0
        self.real_car_data["plan_pos_orientation"] = []
        self.real_car_data["last_received_time"] = self.get_clock().now()
        self.real_car_data["received_global_plan_time"] = 1
        self.real_car_data["orientation_points"] = None
        """
        確定以下資料都有收到 才會在 check_and_get_lastest_data() 更新最新資料
        amcl 追蹤車體目前位置
        goal 目標位置
        lidar lidar資料
        """
        self.data_updated = {
            "amcl": False,
            "goal": False,
            "lidar": False,
        }

        """
        接收目標, 車體目前位置, 輪子轉速, lidar的距離和vector
        """
        self.subscriber_amcl = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.subscribe_callback_amcl, 10
        )
        """
        清空amcl
        """
        self.publisher_clear_amcl = self.create_publisher(
            PoseWithCovarianceStamped, "/amcl_pose", 10
        )
        """
        目標座標
        """
        self.subscriber_goal = self.create_subscription(
            PoseStamped, "/goal_pose", self.subscribe_callback_goal, 1
        )
        """
        lidar掃描
        """
        self.subscriber_lidar = self.create_subscription(
            LaserScan, "/scan", self.laser_scan_callback, 1
        )
        """
        車體後輪電壓
        """
        self.subscriber_rear = self.create_subscription(
            String, DeviceDataTypeEnum.car_C_state, self.rear_wheel_callback, 1
        )
        """
        車體前輪電壓
        """
        self.subscriber_forward = self.create_subscription(
            String, DeviceDataTypeEnum.car_C_state_front, self.forward_wheel_callback, 1
        )

        self.subscriber_target_angle = self.create_subscription(
            Point,
            "/object_coordinates",
            self.subscriber_target_angle_callback,
            10,  # 替换为实际的主题名称
        )
        self.subscriber_object_direction = self.create_subscription(
            String,
            "/object_direction",
            self.subscriber_object_direction_callback,
            10,
        )
        self.subscriber_object_depth = self.create_subscription(
            String,
            "/object_depth",
            self.subscriber_object_depth_callback,
            1,
        )
        self.subscriber_tag_exist = self.create_subscription(
            String,
            "/tag_exist",
            self.subscriber_tag_exist_callback,
            1,
        )
        self.subscriber_arucode_depth = self.create_subscription(
            Float32,
            "/arucode_depth",
            self.subscriber_arucode_depth_callback,
            1,
        )
        self.subscriber_arucode_direction = self.create_subscription(
            String,
            "/arucode_direction",
            self.subscriber_arucode_direction_callback,
            10,
        )
        """
        publish map position ,
        """
        self.publisher_goal_pose = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self.publisher_odom = self.create_publisher(Odometry, "/odom", 10)
        self.publisher_scan = self.create_publisher(LaserScan, "/scan", 10)
        self.publisher_tag_name = self.create_publisher(String, "/tag_name", 10)
        """
        publish給前後的esp32驅動車輪
        """
        self.publisher = self.create_publisher(
            String, DeviceDataTypeEnum.car_C_rear_wheel, 10
        )  # 後輪esp32

        self.publisher_forward = self.create_publisher(
            String, DeviceDataTypeEnum.car_C_front_wheel, 10
        )  # 前輪esp32

        """
        傳送給 unity 通知它做場景 reset 的訊號
        """
        self.publisher_unity_reset_signal = self.create_publisher(
            String, "/reset_signal", 10
        )
        """
        傳送給 unity 通知它目前是 RL mode, 所以 amcl_pose 和 goal_pose 都要從 unity publish amcl_pose 和 goal_pose 的資料
        (此時不能開 foxglove, 不然兩個地圖資訊會互搶)
        """
        self.publisher_unity_RL_signal = self.create_publisher(String, "/RL_signal", 10)
        """
        publish to localization initialpose
        """
        self.publisher_localization_map_signal = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        """
        機械手臂
        """
        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint, DeviceDataTypeEnum.robot_arm, 10
        )

        """
        回傳經過路徑規劃時物體該用多少速度去移動
        return linear x y z & angular x y z
        """
        self.subscriber_navigation = self.create_subscription(
            Twist, "/cmd_vel_nav", self.navigation_callback, 1
        )
        self.last_message_time = time.time()
        self.no_signal = False

        """
        received_global_plan
        """
        # 改成 plan
        self.subscriber_received_global_plan = self.create_subscription(
            Path, "/received_global_plan", self.global_plan_callback, 1
        )


    """
    檢查所有數據是否更新,
    更新最新車體狀態資料
    """

    def check_and_get_lastest_data(self):
        # print(self.data_updated.values())
        if all(self.data_updated.values()):
            # 確認所有的數據都更新並publish
            self.data_updated["amcl"] = False
            self.data_updated["lidar"] = False
            self.lastest_data = preprocess_data(self.real_car_data)

    def get_latest_data(self):
        return self.lastest_data

    def get_target_pos(self):
        return self.real_car_data["arm_tartget_position"]

    def get_object_direction(self):
        return self.real_car_data['object_direction']

    def get_object_depth(self):
        try:
            data = float(self.real_car_data['object_depth'])
        except:
            data = None
        return data

    """
    取得經過data_transform處理後的最新資料

    Returns:
        dict: raw data經過整理的資料

    """
    def publish_tag_name(self, tag_name):
        msg = String()
        msg.data = tag_name
        self.publisher_tag_name.publish(msg)

    def get_tag_exist_signal(self):
        return self.real_car_data["tag_exist"]

    def get_arducode_depth_signal(self):
        return self.real_car_data["arucode_depth"]

    def get_arducode_direction(self):
        return self.real_car_data.get("arucode_direction", None)

    def wait_for_data(self):
        car_state_data = self.get_latest_data()
        while car_state_data is None:
            car_state_data = self.get_latest_data()
        return car_state_data

    def publish_control_signal_rear(self, velocities: List[float], publisher):
        control_signal = {
            "type": str(DeviceDataTypeEnum.car_C_rear_wheel),
            "data": dict(CarCControl(target_vel=velocities)),
        }
        control_msg = String()
        control_msg.data = orjson.dumps(control_signal).decode()
        publisher.publish(control_msg)

    def publish_control_signal_forward(self, velocities: List[float], publisher):
        control_signal = {
            "type": str(DeviceDataTypeEnum.car_C_front_wheel),
            "data": dict(CarCControl(target_vel=velocities)),
        }
        control_msg = String()
        control_msg.data = orjson.dumps(control_signal).decode()
        publisher.publish(control_msg)

    """
    Publish vehicle control signals to robot.

    Args :
    - action(str or [float]) : action code or velocity
    - pid_control : If True, action is treated as PID value
    """

    def publish_to_robot(self, action, pid_control: bool = True):
        if pid_control:
            _vel1, _vel2, _vel3, _vel4 = action
        else:
            mapped_action = ACTION_MAPPINGS.get(
                action, [0, 0, 0, 0]
            )  # Default to stop if action is invalid
            _vel1, _vel2, _vel3, _vel4 = mapped_action
        velocities_front = [_vel1, _vel2]  # 前面的esp32
        velocities_back = [_vel3, _vel4]  # 後面的esp32
        self.publish_control_signal_forward(velocities_front, self.publisher_forward)
        self.publish_control_signal_rear(velocities_back, self.publisher)

    # def publish_arm_jetson(self, joint_pos):
    #     roslibpy_joint_msg = roslibpy.Message(
    #         {
    #             "positions": [
    #                 float(angle) for angle in joint_pos[:5]
    #             ],  # 只取前5个关节角度
    #             "time_from_start": {"secs": 0, "nsecs": 0},
    #         }
    #     )
    #     self.coordinate_topic.publish(roslibpy_joint_msg)

    def publish_arm(self, joint_pos):
        msg = JointTrajectoryPoint()
        msg.positions = [float(pos) for pos in joint_pos]
        msg.velocities = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]  # Replace with actual desired velocities
        self.joint_trajectory_publisher_.publish(msg)

    """
    給車子四個輪子停止的電壓
    """

    def publish_to_robot_reset(self):
        self.publish_to_robot("STOP", pid_control=False)

    """
    重製lastest_data
    """

    def reset(self):
        self.lastest_data = None
        # self.publish_to_robot("STOP", pid_control=False)

    """
    傳送給 unity 車子回去到原點的訊號
    """

    def reset_unity(self):
        msg = String()
        msg.data = "1"
        self.publisher_unity_reset_signal.publish(msg)
        # self.publisher_unity_reset_signal

    def RL_mode_unity(self):
        msg = String()
        msg.data = "1"
        self.publisher_unity_RL_signal.publish(msg)

    """
    確定amcl goal lidar其中一個有收到訊號
    並更新目前是否有接收到的狀態
    """

    def update_and_check_data(self, data_type):
        self.data_updated[data_type] = True
        self.check_and_get_lastest_data()

    """
    amcl的subscribe function
    """

    def subscribe_callback_amcl(self, message):
        pose = self.transfer_car_pose(message)[0]
        quaternion = self.transfer_car_pose(message)[1]
        self.real_car_data["ROS2CarPosition"] = pose
        self.real_car_data["ROS2CarQuaternion"] = quaternion
        self.update_and_check_data("amcl")

    """
    給 subscribe_callback_amcl 做轉換使用
    """

    def transfer_car_pose(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        return [position.x, position.y, position.z], [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ]

    """
    goal 的 subscribe function
    """

    def subscribe_callback_goal(self, message):
        target = self.transfer_target_pos(message)
        if target:
            self.real_car_data["ROS2TargetPosition"] = target
            self.update_and_check_data("goal")

    """
    給 subscribe_callback_goal 轉換用
    """

    def transfer_target_pos(self, msg):
        position = msg.pose.position
        return [position.x, position.y, position.z]

    """
    lidar 的 subscriber function
    """

    def laser_scan_callback(self, msg):
        """
        普通lidar 預設輸出1800個數值
        高級lidar 預設輸出3240個數值
        可以透過print(len(msg.ranges))去觀察
        """
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        ranges_180 = []
        direction_180 = []
        all_ranges = msg.ranges
        for i in range(len(all_ranges)):
            if i % LIDAR_PER_SECTOR == 0:  # handle the amount of lidar.
                angle_tmp = angle_min + i * angle_increment
                ranges_180.append(all_ranges[i])
                direction_180.append([math.cos(angle_tmp), math.sin(angle_tmp), 0])
        combined_lidar_data = (
            [ranges_180[i] for i in FRONT_LIDAR_INDICES]
            + [ranges_180[i] for i in LEFT_LIDAR_INDICES]
            + [ranges_180[i] for i in RIGHT_LIDAR_INDICES]
        )
        self.real_car_data["ROS2Range"] = combined_lidar_data
        self.real_car_data["ROS2RangePosition"] = direction_180
        self.update_and_check_data("lidar")

    """
    navigation輸出路徑所需速度及轉角的subscribe function
    """

    def navigation_callback(self, msg):
        self.last_message_time = time.time()
        self.real_car_data["cmd_vel_nav"] = msg

    """
    navigation的global route planning
    會輸出座標點
    """

    def global_plan_callback(self, msg):
        self.real_car_data["plan_pos_orientation"] = [msg.poses[0].pose.orientation.z, msg.poses[0].pose.orientation.w]
        self.real_car_data["last_received_time"] = self.get_clock().now()
        current_first_point = (msg.poses[0].pose.position.x, msg.poses[0].pose.position.y, msg.poses[0].pose.position.z)
        path_length = len(msg.poses)
        orientation_points = []
        coordinates = []
        if path_length > 0:
            last_recorded_point = msg.poses[0].pose.position
            orientation_points.append((msg.poses[0].pose.orientation.z, msg.poses[0].pose.orientation.w))
            coordinates.append((msg.poses[0].pose.position.x, msg.poses[0].pose.position.y))
            for i in range(1, path_length):
                current_point = msg.poses[i].pose.position
                distance = math.sqrt(
                    (current_point.x - last_recorded_point.x) ** 2 +
                    (current_point.y - last_recorded_point.y) ** 2
                )
                if distance >= 0.3:
                    orientation_points.append((msg.poses[i].pose.orientation.z, msg.poses[i].pose.orientation.w))
                    coordinates.append((current_point.x, current_point.y))
                    last_recorded_point = current_point
        self.real_car_data["orientation_points"] = orientation_points
        self.real_car_data["coordinates"] = coordinates


        # try:
        #     if len(msg.poses) > 1:
        #         current_x, current_y = (
        #             self.real_car_data["ROS2CarPosition"][0],
        #             self.real_car_data["ROS2CarPosition"][1],
        #         )
        #         #  找出離目前車體 NEXT_POINT_DISTANCE 距離的座標點
        #         for i in range(20):
        #             point_x, point_y = (
        #                 msg.poses[i].pose.position.x,
        #                 msg.poses[i].pose.position.y,
        #             )
        #             distance = math.sqrt(
        #                 (point_x - current_x) ** 2 + (point_y - current_y) ** 2
        #             )
        #             if abs(distance - NEXT_POINT_DISTANCE) < 0.01:
        #                 break
        #         self.real_car_data["received_global_plan"] = [point_x, point_y]
        #     else:
        #         self.get_logger().info("Global plan does not contain enough points.")
        # except:
        #     self.real_car_data["received_global_plan"] = None

    def check_plan_update(self):
        current_time = self.get_clock().now()
        last_received_time = self.real_car_data["last_received_time"]

        if last_received_time is None:
            return 1
            self.get_logger().warn("No global plan received yet.")
        else:
            time_diff = (current_time - last_received_time).nanoseconds / 1e9  # 轉換為秒
            if time_diff > 2:  # 假設 5 秒沒有更新
                self.get_logger().warn("Global plan is not updating.")
                return 1
            else:
                return 0
    """
    在navigation沒送資料時讓車子停止
    """

    def check_signal(self):
        timeout = 2.0
        current_time = time.time()
        if current_time - self.last_message_time > timeout:
            self.publish_to_robot_reset()
            return True

    """
    接收後輪的電壓 subscribe function
    """

    def rear_wheel_callback(self, message):
        try:
            json_str = message.data
            data = json.loads(json_str)
            if "data" in data and "vels" in data["data"]:
                vels = data["data"]["vels"]
                self.real_car_data["ROS2WheelAngularVelocityLeftBack"] = vels[0]
                self.real_car_data["ROS2WheelAngularVelocityRightBack"] = vels[1]
            else:
                print("Invalid message format. Missing 'vels' key.")
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")

    """
    接收前輪的電壓 subscribe function
    """

    def forward_wheel_callback(self, message):
        try:
            json_str = message.data
            data = json.loads(json_str)
            if "data" in data and "vels" in data["data"]:
                vels = data["data"]["vels"]
                self.real_car_data["ROS2WheelAngularVelocityLeftFront"] = vels[0]
                self.real_car_data["ROS2WheelAngularVelocityRightFront"] = vels[1]
            else:
                print("Invalid message format. Missing 'vels' key.")
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")

    def subscriber_target_angle_callback(self, msg):
        # self.real_car_data["arm_tartget_position"]
        point_as_list = [msg.x, msg.y, msg.z]
        self.real_car_data["arm_tartget_position"] = point_as_list

    def subscriber_object_direction_callback(self, msg):
        self.real_car_data['object_direction'] = msg.data

    def subscriber_object_depth_callback(self, msg):
        self.real_car_data['object_depth'] = msg.data

    def subscriber_tag_exist_callback(self, msg):
        self.real_car_data["tag_exist"] = msg.data

    def subscriber_arucode_depth_callback(self, msg):
        self.real_car_data["arucode_depth"] = msg.data

    def subscriber_arucode_direction_callback(self, msg):
        self.real_car_data["arucode_direction"] = msg.data

    def reset_amcl(self):
        reset_pose = PoseWithCovarianceStamped()
        reset_pose.header.stamp = self.get_clock().now().to_msg()
        reset_pose.header.frame_id = "map"  # 或者你需要的坐标系
        reset_pose.pose.pose.position.x = float("nan")
        reset_pose.pose.pose.position.y = float("nan")
        reset_pose.pose.pose.position.z = float("nan")
        reset_pose.pose.pose.orientation.x = 0.0
        reset_pose.pose.pose.orientation.y = 0.0
        reset_pose.pose.pose.orientation.z = 0.0
        reset_pose.pose.pose.orientation.w = 1.0
        reset_pose.pose.covariance = [
            float("nan"),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float("nan"),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float("nan"),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float("nan"),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float("nan"),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float("nan"),
        ]
        self.publisher_clear_amcl.publish(reset_pose)
        # self.get_logger().info("Published reset pose to /amcl_pose")

    # 清空scan
    def reset_laser_scan(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "laser_frame"  # 根據實際的 frame_id 設置
        scan_msg.angle_min = -3.14  # 根據實際激光雷達設置
        scan_msg.angle_max = 3.14
        scan_msg.angle_increment = 0.01
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.0
        scan_msg.range_max = 10.0
        scan_msg.ranges = [float("inf")] * int(
            (scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment
        )
        scan_msg.intensities = [0.0] * len(scan_msg.ranges)

        self.publisher_scan.publish(scan_msg)
        self.get_logger().info("LaserScan has been reset")

    # 清空 base_footprint
    def reset_base_footprint(self):
        broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        self.get_logger().info("Base footprint has been reset")

    # 清空 odom
    def reset_odom(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        self.publisher_odom.publish(odom_msg)
        self.get_logger().info("Odom has been reset")

    """
    給 publisher_localization_map 用的 func
    """

    def publish_initial_map_position(self, position, orientation, covariance):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = position["x"]
        msg.pose.pose.position.y = position["y"]
        msg.pose.pose.position.z = position["z"]
        msg.pose.pose.orientation.x = orientation["x"]
        msg.pose.pose.orientation.y = orientation["y"]
        msg.pose.pose.orientation.z = orientation["z"]
        msg.pose.pose.orientation.w = orientation["w"]
        msg.pose.covariance = covariance
        self.publisher_localization_map_signal.publish(msg)
        self.get_logger().info("Publish initial map position")

    def get_initial_position_and_orientation(self):
        return (
            {"x": 0.29377966745215334, "y": -0.327070701568549, "z": 0.0},
            {"x": 0.0, "y": 0.0, "z": 0.045576476174623244, "w": 0.9989608524959847},
            [
                0.25,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.25,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.06853892060437211,
            ],
        )

    """
    先清空所以資料後做 localization
    """

    def publisher_localization_map(self):
        # self.publisher_clear_localization_map()
        # self.reset_all()
        # time.sleep(0.5)

        # self.reset_odom()
        # self.reset_base_footprint()
        # # self.reset_laser_scan()
        # self.reset_amcl()
        # self.publisher_clear_localization_map()
        # time.sleep(1)
        position, orientation, covariance = self.get_initial_position_and_orientation()
        self.publish_initial_map_position(position, orientation, covariance)

    """
    Reset map localization
    """

    def publisher_clear_localization_map(self):
        empty_msg = PoseWithCovarianceStamped()
        empty_msg.header.frame_id = "map"
        empty_msg.pose.pose.position.x = float("nan")
        self.publisher_localization_map_signal.publish(empty_msg)
        self.get_logger().info("Cleared initial map position")

    def read_yaml_pgm_data(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        map_dir = os.path.abspath(os.path.join(script_dir, "../map"))
        yaml_file_path = os.path.join(map_dir, "map01.yaml")
        pgm_file_path = os.path.join(map_dir, "map01.pgm")
        # print("yaml_file_path : ", yaml_file_path)
        # print("pgm_file_path : ", pgm_file_path)
        with open(yaml_file_path, "r") as yaml_file:
            self.map_config = yaml.safe_load(yaml_file)
        resolution = self.map_config["resolution"]
        origin = self.map_config["origin"][:2]  # 只需要 x 和 y 座標
        return resolution, origin, pgm_file_path

    def find_free_coordinates(self, pgm_file_path, resolution, origin):
        # 打開 .pgm 文件
        image = Image.open(pgm_file_path)
        pixels = image.load()
        width, height = image.size

        free_coordinates = []

        # 找到所有的非障礙物座標（假設障礙物是黑色(0)）
        for y in range(height):
            for x in range(width):
                if pixels[x, y] > 0:  # 非障礙物像素（非黑色）
                    # 將像素座標轉換為地圖座標
                    map_x = origin[0] + x * resolution
                    map_y = origin[1] + y * resolution
                    free_coordinates.append((map_x, map_y))

        # 隨機選擇一個非障礙物座標
        if free_coordinates:
            return random.choice(free_coordinates)
        else:
            return None

    def publish_goal_pose(self, position):
        goal_pose = PoseStamped()
        goal_pose.header = Header()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = position[0]
        goal_pose.pose.position.y = position[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0  # 假設沒有特定的方向需求，設置為默認值
        self.publisher_goal_pose.publish(goal_pose)

    def publisher_random_goal_pose(self):
        (resolution, origin, pgm_file_path) = self.read_yaml_pgm_data()
        random_free_coordinate = self.find_free_coordinates(
            pgm_file_path, resolution, origin
        )
        if random_free_coordinate:
            goal_pose = PoseStamped()
            goal_pose.header = Header()
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position.x = random_free_coordinate[0]
            goal_pose.pose.position.y = random_free_coordinate[1]
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0  # 假設沒有特定的方向需求，設置為默認值

            self.publisher_goal_pose.publish(goal_pose)
            self.get_logger().info(
                f"Published random goal pose: {random_free_coordinate}"
            )
        else:
            self.get_logger().warn("No free coordinate found to publish")
