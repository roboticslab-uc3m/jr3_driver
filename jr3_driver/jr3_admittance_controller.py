import rclpy

from .Jr3BaseNode import Jr3BaseNode
from rcl_interfaces.msg import ParameterDescriptor

class Jr3AdmittanceController(Jr3BaseNode):
    def __init__(self):
        super().__init__('jr3_admittance_controller')
        self.get_logger().info('Starting JR3 admittance controller.')

        translation_factor_param = self.declare_parameter('translation_factor', 0.0,
            ParameterDescriptor(description='translation factor for admittance control'))
        rotation_factor_param = self.declare_parameter('rotation_factor', 0.0,
            ParameterDescriptor(description='rotation factor for admittance control'))

        self.translation_factor = translation_factor_param.get_parameter_value().double_value
        self.rotation_factor = rotation_factor_param.get_parameter_value().double_value

        self.get_logger().info('JR3 admittance controller is running.')

    def send_command(self, wrench_0, wrench_tcp, H_0_tcp):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = Jr3AdmittanceController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
