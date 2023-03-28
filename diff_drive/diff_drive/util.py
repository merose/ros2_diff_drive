from builtin_interfaces.msg import Time

from rclpy.parameter import Parameter

from rcl_interfaces.msg import ParameterDescriptor


class BaseNode:

    _PARAMETER_TYPE_MAP = {
        bool: Parameter.Type.BOOL.value,
        float: Parameter.Type.DOUBLE.value,
        int: Parameter.Type.INTEGER.value
    }

    def __init__(self, node):
        self.node = node

    def get_node(self):
        return self.node

    def get_stamp(self):
        return self.node.get_clock().now().to_msg()

    def get_time(self):
        return self.time_from_stamp(self.get_stamp())

    def declare_parameter(self, name, default=None, value_type=None,
                          description=''):
        python_type = value_type if value_type is not None \
            else type(default) if default is not None \
            else str
        param_type = self._PARAMETER_TYPE_MAP[python_type] \
            if python_type in self._PARAMETER_TYPE_MAP \
            else Parameter.Type.STRING.value
        descriptor = ParameterDescriptor(type=param_type,
                                         description=description)
        self.node.declare_parameter(name, value=default, descriptor=descriptor)

    def get_string_parameter(self, name):
        return self.node.get_parameter(name).get_parameter_value().string_value

    def get_integer_parameter(self, name):
        return self.node.get_parameter(name).get_parameter_value().integer_value

    def get_double_parameter(self, name):
        return self.node.get_parameter(name).get_parameter_value().double_value

    def log_debug(self, msg):
        self.node.get_logger().debug(msg)

    def log_info(self, msg):
        self.node.get_logger().info(msg)

    def log_warning(self, msg):
        self.node.get_logger().warning(msg)

    def log_error(self, msg):
        self.node.get_logger().error(msg)

    def time_from_stamp(self, stamp):
        return stamp.sec + stamp.nanosec/1E9

    def stamp_from_time(self, timestamp):
        stamp = Time()
        stamp.sec = int(timestamp)
        stamp.nanosec = (timestamp - stamp.sec) * 1E9
