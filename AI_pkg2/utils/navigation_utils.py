from ros_receive_and_data_processing.config import BODY_WIDTH, WHEEL_DIAMETER, FACTOR
from math import pi
from typing import Tuple

"""
Calculate wheel speeds based on the given command velocity.

Parameters :
    cmd_vel_nav: ROS2 Topic data

Returns :
    A tuple of two floats representing the left and right wheel speeds in PID (revolutions per minute).

Example :
    car_data = self.node.wait_for_data()
    self.pwm_left, self.pwm_right = self.calculate_wheel_speeds(car_data["navigation_data"])
"""


def calculate_wheel_speeds(cmd_vel_nav) -> Tuple[float, float]:
    linear_velocity = cmd_vel_nav.linear.x
    angular_velocity = cmd_vel_nav.angular.z
    L = BODY_WIDTH
    v_left = linear_velocity - (L / 2) * angular_velocity
    v_right = linear_velocity + (L / 2) * angular_velocity
    rpm_left, rpm_right = speed_to_rpm(v_left), speed_to_rpm(v_right)
    pid_left, pid_right = rpm_to_pid(rpm_left), rpm_to_pid(rpm_right)
    return pid_left, pid_right


"""
根據WHEEL_DIAMETER轉換成RPM
"""


def speed_to_rpm(speed) -> float:
    wheel_circumference = pi * WHEEL_DIAMETER
    return (speed / wheel_circumference) * 60


"""
rpm轉換成PID數值, 可於esp32程式碼做測量, 找尋rpm與PID數值之間的關係
"""


def rpm_to_pid(rpm) -> float:
    return rpm / FACTOR


"""
根據曲線選擇動作
以下數值為車頭偏離要到達的目標角度差
"""


def action_choice(angle_to_target):
    if abs(angle_to_target) < 30:
        action = "FORWARD"
    # elif 30 < abs(angle_to_target) < 50:
    #     if angle_to_target > 0:
    #         action = "LEFT_FRONT"
    #     elif angle_to_target < 0:
    #         action = "RIGHT_FRONT"
    else:
        if angle_to_target > 0:
            action = "COUNTERCLOCKWISE_ROTATION"
        elif angle_to_target < 0:
            action = "CLOCKWISE_ROTATION"
    return action
