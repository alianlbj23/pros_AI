from utils.navigation_utils import calculate_wheel_speeds, action_choice
from utils.rotate_angle import calculate_angle_to_target, get_yaw_from_quaternion, calculate_angle_point
from utils.obs_utils import cal_distance, calculate_dynamic_indices
import threading
from robot_arm.robot_control import RobotArmControl
from csv_store_and_file.csv_store import DataCollector
from ros_receive_and_data_processing.config import TARGET_DISTANCE, FRONT_LIDAR_INDICES, LEFT_LIDAR_INDICES, RIGHT_LIDAR_INDICES
import random

class NavigationProcess:
    def __init__(self, node):
        self.node = node
        self.robot_controler = RobotArmControl(
            node,
        )
        self.flag = 1
        # self.data_collector = DataCollector()  # 資料收集

    def robot_action_thread(self):
        self.robot_controler.action()

    """
    接收車子經處理過的state
    """

    def node_receive_data(self):
        self.node.reset()
        return self.node.wait_for_data()

    def process_car_data(self, car_data):
        received_global_plan = car_data["received_global_plan"]
        car_position = car_data["car_pos"][:2]
        pid_left, pid_right = calculate_wheel_speeds(car_data["cmd_vel_nav"])
        return received_global_plan, car_position, pid_left, pid_right

    # 到達終點時要做的事
    def handle_reached_destination(self):
        print("end")
        action = "STOP"
        # 讓機械手臂動
        # robot_thread = threading.Thread(target=self.robot_action_thread)
        # robot_thread.start()
        self.node.publish_to_robot(action, pid_control=False)

    def rule(self, car_data, safe_distance):
        obstacle_near = any(lidar < safe_distance for lidar in car_data["lidar_data"])
        if obstacle_near:
            front_clear = all(car_data["lidar_data"][i] > safe_distance for i in FRONT_LIDAR_INDICES)
            left_clear = all(car_data["lidar_data"][i] > safe_distance for i in LEFT_LIDAR_INDICES)
            right_clear = all(car_data["lidar_data"][i] > safe_distance for i in RIGHT_LIDAR_INDICES)
            if front_clear:
                action = "FORWARD"
                self.node.publish_to_robot(action, pid_control=False)
            elif left_clear:
                action = "COUNTERCLOCKWISE_ROTATION"
                self.node.publish_to_robot(action, pid_control=False)
            elif right_clear:
                action = "CLOCKWISE_ROTATION"
                self.node.publish_to_robot(action, pid_control=False)
        else:
            self.simple_handle_action(car_data)

    def simple_handle_action(self, car_data):
        print("simple")
        action = action_choice(car_data['angle_diff'])
        self.node.publish_to_robot(action, pid_control=False)

    def handle_action_plus(self, car_data):
        orientation_points = self.node.real_car_data.get("orientation_points", None)
        coordinates = self.node.real_car_data.get("coordinates", None)

        if not orientation_points or not coordinates:
            action = "STOP"
            self.node.publish_to_robot(action, pid_control=False)
            return

        previous_length = len(orientation_points)

        for i, (z, w) in enumerate(orientation_points):
            new_orientation_points = self.node.real_car_data.get("orientation_points", None)
            new_coordinates = self.node.real_car_data.get("coordinates", None)
            current_length = len(new_orientation_points) if new_orientation_points else 0

            if new_orientation_points and current_length != previous_length:
                orientation_points = new_orientation_points
                coordinates = new_coordinates
                previous_length = current_length
                i = 0
                continue

            plan_yaw = get_yaw_from_quaternion(z, w)
            car_yaw = get_yaw_from_quaternion(car_data["car_quaternion"][0], car_data["car_quaternion"][1])

            diff_angle = (plan_yaw - car_yaw) % 360.0
            if diff_angle < 10.0 or (diff_angle > 350 and diff_angle < 360):
                action = "FORWARD"
            elif diff_angle > 10.0 and diff_angle < 180.0:
                action = "COUNTERCLOCKWISE_ROTATION"
            elif diff_angle > 180.0 and diff_angle < 350.0:
                action = "CLOCKWISE_ROTATION"
            else:
                action = "STOP"
            if self.node.check_plan_update == 1:
                if action != "STOP":
                    action += "_SLOW"
            self.node.publish_to_robot(action, pid_control=False)

            current_car_pos = self.node.real_car_data.get("ROS2CarPosition", [0, 0])[:2]
            distance_to_current_point = cal_distance(current_car_pos, coordinates[i][:2])

            # 如果到达当前路径点，继续处理下一个路径点
            if distance_to_current_point < 0.05:
                continue  # 处理下一个路径点
            else:
                break  # 尚未到达当前路径点，停止处理后续点，等待下一次调用









    # 未到達終點時做的事
    def handle_action(self, car_data):
        self.flag = 1
        """
        stop_signal == True 時代表 nav2 的 cmd_vel_nav 的 topic 沒訊號, 要改用其他控制方式
        車子接收 nav2 的 cmd_vel_nav 的速度移動快碰到牆壁時會花很長時間重算, 因為要等很久
        , 所以發現他在重算就改用其他控制方式
        """
        stop_signal = self.node.check_signal()
        received_global_plan, car_position, pid_left, pid_right = self.process_car_data(
            car_data
        )
        """
        因為 received_global_plan 有時會讀取到空值的關係, 若讀取到空值就讓車子停止
        """
        # print("car_data['car_target_distance'] : ", car_data["car_target_distance"])

        # else:
            # safe_distance = 0.3
            # print("min lidar : ", min(car_data['lidar_data']))
            # if min(car_data['lidar_data']) < safe_distance:
            #     self.rule(car_data, safe_distance)
            # else:
        # elif self.node.check_plan_update():
        #     print("rule")
        #     self.rule(car_data, 0.5)
        #     # action = "STOP"
        #     # self.node.publish_to_robot(action, pid_control=False)
        # else:

        # if self.node.check_plan_update:
        #     print("no signal")
        #     action = "STOP"
        #     self.node.publish_to_robot(action, pid_control=False)
        # else:
        if car_data["car_target_distance"] > 0.1 and car_data["car_target_distance"] < 0.5:
            print("into simple mode")
            self.simple_handle_action(car_data)
        else:
            try:
                plan_yaw = get_yaw_from_quaternion(self.node.real_car_data["plan_pos_orientation"][0],self.node.real_car_data["plan_pos_orientation"][1])
                car_yaw = get_yaw_from_quaternion(car_data["car_quaternion"][0],car_data["car_quaternion"][1])
                diff_angle = (plan_yaw - car_yaw) % 360.0

                if diff_angle < 20.0 or (diff_angle > 340 and diff_angle < 360):
                    action = "FORWARD"
                    self.node.publish_to_robot(action, pid_control=False)
                elif diff_angle > 20.0 and diff_angle < 180.0:
                    action = "COUNTERCLOCKWISE_ROTATION"
                    self.node.publish_to_robot(action, pid_control=False)
                elif diff_angle > 180.0 and diff_angle < 340.0:
                    action = "CLOCKWISE_ROTATION"
                    self.node.publish_to_robot(action, pid_control=False)
            except:
                    # print("stop")
                    action = "STOP"
                    self.node.publish_to_robot(action, pid_control=False)
        # if received_global_plan == None:
        #     action = "STOP"
        #     self.node.publish_to_robot(action, pid_control=False)
        # else:
        #     """
        #     根據車頭面向全域路徑給的路徑距離(config設定的NEXT_POINT_DISTANCE)座標計算角度差，
        #     用角度差決定要用什麼動作
        #     """
        #     angle_to_target = calculate_angle_to_target(
        #         car_position, received_global_plan, car_data["car_quaternion"]
        #     )
        #     # 目前車子的位置與 nav2 提供的下一個目標點之間的偏離程度，可以用來計算車頭的偏差，從而決定下一步的行動
        #     action = action_choice(angle_to_target)
        #     # self.node.publish_to_robot(action, pid_control=False)

    def run(self):
        # 車子距離終點一定的距離時便判定到達目標
        car_data = self.node_receive_data()
        while car_data["car_target_distance"] > TARGET_DISTANCE:
            car_data = self.node_receive_data()
            self.handle_action(car_data)
        self.handle_reached_destination()

    def nav_to_target(self, target_position):
        self.node.publish_goal_pose(target_position)
        car_data = self.node_receive_data()
        while car_data["car_target_distance"] > TARGET_DISTANCE:
            random_number = round(random.uniform(0.00, 1.00), 3) / 10.0
            # print(random_number)
            target_position[0] += random_number
            target_position[1] += random_number
            self.node.publish_goal_pose(target_position)
            car_data = self.node_receive_data()
            self.handle_action(car_data)
        self.handle_reached_destination()

    def simple_nav_to_target(self, target_position):
        self.node.publish_goal_pose(target_position)
        while car_data["car_target_distance"] > TARGET_DISTANCE:
            self.simple_handle_action(car_data)
        self.handle_reached_destination()

    def nav_to_target_plus(self, target_position):
        while True:
            self.node.publish_goal_pose(target_position)
            car_data = self.node_receive_data()

            if car_data["car_target_distance"] > 0.1 and car_data["car_target_distance"] < 0.7:
                self.simple_handle_action(car_data)
            else:
                self.handle_action_plus(car_data)
            if car_data["car_target_distance"] <= 0.3:
                break



        self.handle_reached_destination()
