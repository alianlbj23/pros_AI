"""
會先刪除以前的 data 後將不需要的 key 移除, 只留下自己想給 RL 看的
"""


def get_observation(AI_node):
    AI_node.reset()
    data_dict = remove_dict_key(AI_node.wait_for_data())
    return data_dict


def remove_dict_key(data_dict):
    keys_to_remove = [
        "car_pos",
        "target_pos",
        "car_quaternion",
        "car_target_distance",
        "cmd_vel_nav",
        "relative_coordinates",
        "received_global_plan",
        "angle_diff",
    ]
    for key in keys_to_remove:
        data_dict.pop(key, None)  # 使用 pop 方法，避免 KeyError
    return data_dict
