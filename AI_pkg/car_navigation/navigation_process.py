from utils.navigation_utils import calculate_wheel_speeds, action_choice
from utils.rotate_angle import calculate_angle_to_target, get_yaw_from_quaternion
from utils.obs_utils import cal_distance
import threading
from robot_arm.robot_control import RobotArmControl
from csv_store_and_file.csv_store import DataCollector
from ros_receive_and_data_processing.config import TARGET_DISTANCE


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
        if self.flag == 1:
            print("end")
            action = "STOP"
            # 讓機械手臂動
            # robot_thread = threading.Thread(target=self.robot_action_thread)
            # robot_thread.start()
            self.node.publish_to_robot(action, pid_control=False)
            self.flag = 0

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
        # pid_left *= 2
        # pid_right *= 2

        # 接收到 cmd_vel_nav 的訊號，將以下PID數值傳送至車子的PID控制器
        # distance = cal_distance(car_data['car_pos'], car_data['target_pos'])


        # elif stop_signal is not True:
        #     self.node.publish_to_robot(
        #         [pid_left, pid_right, pid_left, pid_right],
        #         pid_control=True,
        #     )
        # 沒接收到 cmd_vel_nav 訊號，因此改用在 ros_receive_and_data_processing 內 confitg 自訂的 action
        # else:
        """
        因為 received_global_plan 有時會讀取到空值的關係, 若讀取到空值就讓車子停止
        """
        if car_data["car_target_distance"] < 0.4:
            angle_to_target = calculate_angle_to_target(
                car_data['car_pos'], car_data['target_pos'], car_data["car_quaternion"]
            )
            action = action_choice(angle_to_target)
            self.node.publish_to_robot(action, pid_control=False)
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
                    print("stop")
                    action = "STOP"
                    self.node.publish_to_robot(action, pid_control=False)
                    # pass
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
            self.node.publish_goal_pose(target_position)
            car_data = self.node_receive_data()
            self.handle_action(car_data)
        self.handle_reached_destination()
