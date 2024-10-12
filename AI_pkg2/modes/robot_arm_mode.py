from controllers.robot_arm_controller import RobotArmController

def run(node):
    controller = RobotArmController(node)
    controller.object_depth_test("fire")