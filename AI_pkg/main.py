import rclpy
import threading
from ros_receive_and_data_processing.AI_node import AI_node
from avoidance_rule.rule_base import RuleBasedController

# from car_supervised.lstm_inference import supervised_inference
from car_navigation.navigation_main import NavigationController
from car_navigation.navigation_process import NavigationProcess
from robot_arm.robot_control import RobotArmControl

# RL
# import gymnasium as gym
# from stable_baselines3 import PPO, DDPG
# from stable_baselines3.common.noise import NormalActionNoise
# from stable_baselines3.common.env_util import make_vec_env
# from car_RL.RL_training_main import CustomCarEnv
# from car_RL.custom_callback import CustomCallback
# from stable_baselines3.common.logger import configure
# from stable_baselines3.common.monitor import Monitor
# import numpy as np


# def load_or_create_model_DDPG(env, model_path):
#     n_actions = env.action_space.shape[-1]
#     action_noise = NormalActionNoise(
#         mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions)
#     )

#     try:
#         model = DDPG.load(model_path)  # Load model
#         model.set_env(env)
#         print(f"Model loaded successfully from {model_path}")
#         print(f"Model learning rate: {model.lr_schedule(1.0)}")
#         print(f"Model policy network: {model.policy}")
#     except FileNotFoundError:  # If not found, train a new one
#         model = DDPG(
#             "MlpPolicy",
#             env,
#             action_noise=action_noise,
#             verbose=1,
#             learning_rate=0.001,
#             device="cuda",
#         )
#         print("Model not found. Training a new model.")
#     return model

# def load_or_create_model_PPO(env, model_path):
#     try:
#         model = PPO.load(model_path)  #  load model
#         env = Monitor(env)
#         model.set_env(env)
#         print(f"Model loaded successfully from {model_path}")
#         print(f"Model learning rate: {model.lr_schedule(1.0)}")
#         print(f"Model policy network: {model.policy}")
#     except FileNotFoundError:  #  找不到就重新train一個
#         model = PPO("MlpPolicy", env, verbose=1, learning_rate=0.001, device="cuda")
#         print("Model not found. Training a new model.")
#     return model

# def train_model_PPO(env):
#     model = load_or_create_model_PPO(
#         env, "./Model/ppo_custom_car_model_1000_1721272714.460118"
#     )
#     custom_callback = CustomCallback("./Model/ppo_custom_car_model", save_freq=1000)
#     total_timesteps = 100000  # 訓練回合數
#     model.learn(
#         total_timesteps=total_timesteps, callback=custom_callback, log_interval=1
#     )  #  進入env開始訓練


# def train_model_DDPG(env):
#     model = load_or_create_model_DDPG(
#         env, "./Model/ddpg_custom_car_model_dsad1000_1721284347.637723"
#     )
#     custom_callback = CustomCallback("./Model/ddpg_custom_car_model", save_freq=1000)
#     total_timesteps = 100000  # 訓練回合數
#     model.learn(
#         total_timesteps=total_timesteps, callback=custom_callback, log_interval=1
#     )  #  進入env開始訓練


# def gym_env_register(AI_node):
#     gym.register(
#         id=CustomCarEnv.ENV_NAME,  # 使用自定義環境的名稱
#         entry_point="car_RL.RL_training_main:CustomCarEnv",  # 模組名稱:類名稱
#     )
#     return gym.make("CustomCarEnv-v0", AI_node=AI_node)


def init_ros_node():
    """node初始化並開一個thread跑ros node"""
    rclpy.init()
    node = AI_node()
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()
    return node, thread


def main(mode):
    node, ros_thread = init_ros_node()

    if mode == "1":
        rule_controller = RuleBasedController(
            node,
        )
        rule_controller.run()
    elif mode == "2":
        # navigation_controller = NavigationController(
        #     node,
        # )
        navigation_controller = NavigationProcess(
            node,
        )
        # navigation_controller.run()
        navigation_controller.nav_to_target_plus(target_position=[  1.9762550096385851, 0.3348260662275371])
        navigation_controller.nav_to_target_plus(target_position=[  2.2328382178946735, -1.1671897513857816])
        navigation_controller.nav_to_target_plus(target_position=[   -0.02219769945382244, -0.008019708769910538])
    elif mode == "3":
        robot_controler = RobotArmControl(
            node,
        )
        # robot_controler.action()
        # robot_controler.publish_tag("fire")
        # robot_controler.grap("fire")
        # robot_controler.put_object()
        # robot_controler.forward_grap()
    elif mode == "4":
        navigation_controller = NavigationProcess(
            node,
        )
        robot_controler = RobotArmControl(
            node,
        )
        navigation_controller.nav_to_target(target_position=[  1.9762550096385851, 0.3348260662275371]) #pokemon
        # robot_controler.object_grasping("water")
        navigation_controller.nav_to_target(target_position=[ 2.2328382178946735, -1.1671897513857816]) # arucode position
        # robot_controler.put_object()

        # navigation_controller.nav_to_target(target_position=[1.3708422437463263,  -1.8874063927940004]) # start2
        # navigation_controller.nav_to_target(target_position=[0.2216050593090444,  -1.7692406054393635]) #pokemon
        # robot_controler.object_grasping("fire")
        # navigation_controller.nav_to_target(target_position=[1.3708422437463263,  -1.8874063927940004]) # start2
        # navigation_controller.nav_to_target(target_position=[ 1.6620495642279993, -1.6739744745747895]) # arucode position
        # robot_controler.put_object()

        # navigation_controller.nav_to_target(target_position=[1.3322890942816212,  -2.6646953834223135]) # start3
        # navigation_controller.nav_to_target(target_position=[ 0.37740678749717826,  -2.659768902216764]) # pokemon
        # robot_controler.object_grasping("green")
        # navigation_controller.nav_to_target(target_position=[1.3322890942816212,  -2.6646953834223135]) # start3
        # navigation_controller.nav_to_target(target_position=[ 2.115643965288565, -1.650247959169505]) # arucode position
        # robot_controler.put_object()

        robot_controler.stop_all_action()
        robot_controler.stop_threads()
        # navigation_controller.nav_to_target(target_position=[1.8161189408830603, -0.19898427821375453])
    # elif mode == "4":
    #     env = gym_env_register(node)
    #     # train_model_PPO(env)
    #     train_model_DDPG(env)

    else:
        print("Please type the correct numbers.")

    rclpy.shutdown()
    ros_thread.join()


def print_usage():
    print("modes:")
    print(" 1 -- rule-based.")
    # print(" 2 -- supervised learning inference.")
    print(" 2 -- ros2 navigation.")
    print(" 3 -- ros2 arm.")
    print(" 4 -- Destroy mode.")


if __name__ == "__main__":
    print_usage()
    mode = input("Enter mode: ")
    main(mode)
