from diff_drive.util import BaseNode

import rclpy
from rclpy.node import Node


class SampleNode(BaseNode):

    def __init__(self, node):
        super().__init__(node)
        self.timer = node.create_timer(1, self.timer_callback)
        self.node.declare_parameter('param1', 'value1')
        result = [param.get_parameter_value() for param
                  in self.node.declare_parameters('', [
                      ('param2', 'value2')
                  ])]
        print(f'result: {result}')
        self.declare_parameter('param3', value='value3')

    def timer_callback(self):
        param1 = self.node.get_parameter('param1') \
                          .get_parameter_value() \
                          .string_value
        param2 = self.node.get_parameter('param2') \
                          .get_parameter_value() \
                          .string_value
        param3 = self.get_string_parameter('param3')
        self.log_info(f'got here - param1={param1} param2={param2} param3={param3}')


def main():
    rclpy.init()
    sample = SampleNode(Node('sample'))
    rclpy.spin(sample.get_node())


if __name__ == '__main__':
    main()
