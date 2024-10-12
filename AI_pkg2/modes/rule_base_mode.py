from controllers.rule_base_controller import RuleBasedController
def run(node):
    controller = RuleBasedController(node)
    controller.run()