import time
from controllers.navigation_controller import NavigationController
from controllers.robot_arm_controller import RobotArmController

def run(node):
    nav_controller = NavigationController(node)
    arm_controller = RobotArmController(node)

    tasks = [
        ("nav to pokemon", lambda: nav_controller.nav_to_target_plus(target_position=[2.2391839038128727, -0.12556139737959193])),
        ("find pokemon", lambda: arm_controller.object_grasping("water")),
        # ... (其他任务)
    ]

    for task_name, task_func in tasks:
        print(task_name)
        task_func()
        time.sleep(0.5)

    arm_controller.stop_all_action()
    arm_controller.stop_threads()