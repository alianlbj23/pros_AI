import numpy as np
import math
from geometry_msgs.msg import Twist

"""
data轉成np後給env的observation
"""


def process_data_to_npfloat32_array(unity_data):
    flat_list = []
    for value in unity_data.values():
        if isinstance(value, list):
            flat_list.extend(value)
        elif isinstance(value, Twist):
            # 将 Twist 对象转换为数值
            flat_list.extend(
                [
                    value.linear.x,
                    value.linear.y,
                    value.linear.z,
                    value.angular.x,
                    value.angular.y,
                    value.angular.z,
                ]
            )
        else:
            flat_list.append(value)
    return np.array(flat_list, dtype=np.float32)


"""
自動推算lidar前方 左方 右方的射線範圍
"""


def calculate_dynamic_indices(lidar_range):
    # 根據LIDAR射線總數動態計算各方向的索引範圍
    # 計算每個方向上的射線數量
    front_count = int(lidar_range * 0.3)  # 前方30%
    side_count = front_count  # 左側和右側各30%，因為不考慮後方40%

    # 計算前方的索引範圍，前方被平分為正向和反向兩部分
    front_start_positive = 0
    front_end_positive = front_count // 2
    front_start_negative = -front_end_positive

    # 計算左側的索引範圍
    left_start = front_end_positive
    left_end = left_start + side_count

    # 計算右側的索引範圍
    right_start = lidar_range - side_count
    right_end = lidar_range

    return {
        "front_positive": (front_start_positive, front_end_positive),
        "front_negative": (front_start_negative, 0),
        "left": (left_start, left_end),
        "right": (right_start, right_end),
    }


"""
將小數取到第三位
"""


def round_to_decimal_places(data_list, decimal_places=3):
    return [round(num, decimal_places) for num in data_list]


"""
將數值轉換成float
"""


def trans_to_float(data_list):
    return [float(i) for i in data_list]


"""
計算車子與目標之間的距離
"""


def cal_distance(car_pos, target_pos):
    car_target_distance = (car_pos[0] - target_pos[0]) ** 2 + (
        car_pos[1] - target_pos[1]
    ) ** 2
    car_target_distance = round_to_decimal_places([math.sqrt(car_target_distance)])[0]
    return car_target_distance
