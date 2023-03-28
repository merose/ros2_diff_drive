from rclpy.parameter import Parameter

from rcl_interfaces.msg import ParameterDescriptor


class BaseNode:

    def __init__(self, node):
        self.node = node

    def get_node(self):
        return self.node

    def declare_parameter(self, name, value=None,
                          type=Parameter.Type.NOT_SET.value,
                          description=''):
        descriptor = ParameterDescriptor(type=type, description=description)
        self.node.declare_parameter(name, value=value, descriptor=descriptor)

    def get_string_parameter(self, name):
        return self.node.get_parameter(name).get_parameter_value().string_value

    def get_integer_parameter(self, name):
        return self.node.get_parameter(name).get_parameter_value().integer_value

    def get_double_parameter(self, name):
        return self.node.get_parameter(name).get_parameter_value().double_value

    def log_info(self, msg):
        self.node.get_logger().info(msg)

    def log_warning(self, msg):
        self.node.get_logger().warning(msg)

    def log_error(self, msg):
        self.node.get_logger().error(msg)
