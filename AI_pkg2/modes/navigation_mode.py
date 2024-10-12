from controllers.navigation_controller import NavigationController

def run(node):
    controller = NavigationController(node)
    while True:
        controller.nav_to_target_plus(target_position=[1.9762550096385851, 0.3348260662275371])
        controller.nav_to_target_plus(target_position=[1.8828640643738286, -1.6112252017559352])
        controller.nav_to_target_plus(target_position=[-0.02219769945382244, -0.008019708769910538])