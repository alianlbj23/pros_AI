# config.py

"""
vel、rotate_vel
"""
vel = 20.0
rotate_vel = 5.0

ACTION_MAPPINGS = {
    "FORWARD": [vel, vel, vel, vel],  # 前進
    "LEFT_FRONT": [rotate_vel, rotate_vel * 1.2, rotate_vel, rotate_vel * 1.2],  # 左前
    "COUNTERCLOCKWISE_ROTATION": [
        -rotate_vel,
        rotate_vel,
        -rotate_vel,
        rotate_vel,
    ],  # 左自轉
    "BACKWARD": [-vel, -vel, -vel, -vel],  # 後退
    "CLOCKWISE_ROTATION": [rotate_vel, -rotate_vel, rotate_vel, -rotate_vel],  # 右自轉
    "RIGHT_FRONT": [rotate_vel * 1.2, rotate_vel, rotate_vel * 1.2, rotate_vel],  # 右前
    "STOP": [0.0, 0.0, 0.0, 0.0],
}

"""
LIDAR_RANGE : 設定lidar要有幾個偵測
FRONT_LIDAR_INDICES : 車子偵測前面障礙物的lidar range
LEFT_LIDAR_INDICES : 車子偵測左邊障礙物的lidar range
RIGHT_LIDAR_INDICES : 車子偵測右邊障礙物的lidar range
"""
LIDAR_RANGE = 90
LIDAR_PER_SECTOR = 20
FRONT_LIDAR_INDICES = list(range(0, 16)) + list(
    range(-15, 0)
)  # 0~16個和-15到0的LIDAR RANGE
LEFT_LIDAR_INDICES = list(range(16, 46))
RIGHT_LIDAR_INDICES = list(range(-45, -15))

"""
OBSTACLE_DISTANCE : 開始閃避障礙物的距離(公尺)
WALL_DISTANCE : 判定為撞到牆壁的距離
TARGET_DISTANCE : 判定為成功到達目標的距離
"""
OBSTACLE_DISTANCE = 0.7
WALL_DISTANCE = 0.2
TARGET_DISTANCE = 1

"""
BODY_WIDTH : 車寬
WHEEL_DIAMETER : 車輪直徑
"""
BODY_WIDTH = 0.3
WHEEL_DIAMETER = 0.05

"""
NEXT_POINT_DISTANCE : 距離下一個點的距離
"""
NEXT_POINT_DISTANCE = 0.5
