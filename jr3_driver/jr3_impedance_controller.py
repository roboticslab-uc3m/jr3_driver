import rclpy

from .Jr3BaseNode import Jr3BaseNode
from rcl_interfaces.msg import ParameterDescriptor

class Jr3ImpedanceController(Jr3BaseNode):
    def __init__(self):
        super().__init__('jr3_impedance_controller')
        self.get_logger().info('Starting JR3 impedance controller.')

        linear_stiffness_param = self.declare_parameter('linear_stiffness', 0.0,
            ParameterDescriptor(description='linear stiffness for impedance control'))
        angular_stiffness_param = self.declare_parameter('angular_stiffness', 0.0,
            ParameterDescriptor(description='angular stiffness for impedance control'))
        linear_viscosity_param = self.declare_parameter('linear_viscosity', 0.0,
            ParameterDescriptor(description='linear viscosity for impedance control'))
        angular_viscosity_param = self.declare_parameter('angular_viscosity', 0.0,
            ParameterDescriptor(description='angular viscosity for impedance control'))

        self.linear_stiffness = linear_stiffness_param.get_parameter_value().double_value
        self.angular_stiffness = angular_stiffness_param.get_parameter_value().double_value
        self.linear_viscosity = linear_viscosity_param.get_parameter_value().double_value
        self.angular_viscosity = angular_viscosity_param.get_parameter_value().double_value

        self.get_logger().info('JR3 impedance controller is running.')

    def send_command(self, wrench_0, H_0_tcp):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = Jr3ImpedanceController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
