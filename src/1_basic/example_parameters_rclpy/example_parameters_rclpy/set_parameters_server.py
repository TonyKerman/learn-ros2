from rcl_interfaces.msg import SetParametersResult

import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__('my_node_name')

        # Declare a parameter
        self.declare_parameter("my_param_name", 42)

        # Then create callback
        self.set_parameters_callback(self.callback)
    
    def callback(self, parameters):
        result = SetParametersResult(successful=True)

        for p in parameters:
            if p.name == "my_param_name":
                if p.type_ != p.Type.INTEGER:
                    result.successful = False
                    result.reason = 'my_param_name must be an Integer'
                    return result
                if p.value < 20:
                    result.successful = False
                    result.reason = "my_param_name must be >= 20"
                    return result

        # Return success, so updates are seen via get_parameter()
        return result
