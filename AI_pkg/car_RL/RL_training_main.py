import numpy as np
import time
import gymnasium as gym
from gymnasium import spaces

# from avoidance import refined_obstacle_avoidance_with_target_orientation
from utils.obs_utils import process_data
from utils.RL_utils import get_observation


class CustomCarEnv(gym.Env):
    ENV_NAME = "CustomCarEnv-v0"

    def __init__(self, AI_node):
        super(CustomCarEnv, self).__init__()
        # state初始化
        self.AI_node = AI_node
        # self.start_time = time.time()
        shape_number = self.get_initial_shape()
        self.action_space = spaces.MultiDiscrete([21, 21])
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(shape_number,), dtype=np.float32
        )

    def step(self, action):
        # 0:前進 1:左轉 2:右轉 3:後退

        # elapsed_time = time.time() - self.start_time  #  計時

        self.AI_node.publish_to_unity(action)  #  送出後會等到unity做完動作後

        unity_data = self.AI_node.get_latest_data()

        reward = 1

        self.state = process_data(unity_data)

        terminated = (
            unity_data["car_target_distance"] < 1 or min(unity_data["lidar_data"]) < 0.2
        )
        return self.state, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        # self.AI_node.publish_to_unity_RESET()  #  送結束訊後給unity
        # self.AI_node.reset()
        unity_data_reset_state = self.AI_node.get_latest_data()
        self.state = process_data(unity_data_reset_state)

        # self.start_time = time.time()

        print("Reset Game")
        self.AI_node.reset()
        time.sleep(1)
        return self.state, {}

    def get_initial_shape(self):
        obs_state = get_observation(self.AI_node)
        obs_state = process_data(obs_state)
        return len(obs_state)
