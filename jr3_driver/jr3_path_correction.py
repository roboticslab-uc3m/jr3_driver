import PyKDL as kdl
import rclpy

from .Jr3BaseNode import Jr3BaseNode
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Point

class Jr3PathCorrection(Jr3BaseNode):
    def __init__(self):
        super().__init__('jr3_path_correction')
        self.get_logger().info('Starting JR3 path correction.')

        pressure_x_param = self.declare_parameter('pressure_x', 0.0,
            ParameterDescriptor(description='pressure along x axis for path correction (tool frame)'))
        pressure_y_param = self.declare_parameter('pressure_y', 0.0,
            ParameterDescriptor(description='pressure along y axis for path correction (tool frame)'))
        pressure_z_param = self.declare_parameter('pressure_z', 0.0,
            ParameterDescriptor(description='pressure along z axis for path correction (tool frame)'))
        kp_param = self.declare_parameter('kp', 0.0,
            ParameterDescriptor(description='proportional gain for path correction'))
        ki_param = self.declare_parameter('ki', 0.0,
            ParameterDescriptor(description='integral gain for path correction'))
        kd_param = self.declare_parameter('kd', 0.0,
            ParameterDescriptor(description='derivative gain for path correction'))

        pressure_x = pressure_x_param.get_parameter_value().double_value
        pressure_y = pressure_y_param.get_parameter_value().double_value
        pressure_z = pressure_z_param.get_parameter_value().double_value

        self.pressure = kdl.Vector(pressure_x, pressure_y, pressure_z)
        self.get_logger().info(f'Using pressure vector: [{self.pressure.x()}, {self.pressure.y()}, {self.pressure.z()}] N')

        self.kp = kp_param.get_parameter_value().double_value
        self.ki = ki_param.get_parameter_value().double_value
        self.kd = kd_param.get_parameter_value().double_value
        self.get_logger().info(f'Using PID gains: kp={self.kp}, ki={self.ki}, kd={self.kd}')

        self.proportional_error = kdl.Vector(0.0, 0.0, 0.0)
        self.integral_error = kdl.Vector(0.0, 0.0, 0.0)
        self.initial_fz = None

        self.publisher = self.create_publisher(Point, 'command/path_corr', 10)
        self.get_logger().info('JR3 path correction is running.')

    def send_command(self, wrench_0, wrench_tcp, H_0_tcp):
        if self.initial_fz is None:
            self.initial_fz = wrench_tcp.force.z()
            self.get_logger().info(f'Initial Fz set to: {self.initial_fz} N')

        local_wrench = kdl.Wrench(kdl.Vector.Zero(), kdl.Vector.Zero())
        local_wrench.force.z(wrench_tcp.force.z() - self.initial_fz)
        self.get_logger().info(f'Wrench in TCP frame after zeroing Fz:[{local_wrench.force.z()} N')

        prev_proportional_error = self.proportional_error

        # this is a sum since:
        # - pressure: force to be exerted by the tool on the environment to correct the path
        # - wrench_tcp.force: force measured by the sensor
        proportional_error = local_wrench.force - self.pressure
        proportional_term = self.kp * proportional_error

        if self.ki != 0.0:
            self.integral_error += proportional_error * self.command_period
            integral_term = self.ki * self.integral_error
            self.integral_error = integral_term / self.ki
        else:
            integral_term = kdl.Vector(0.0, 0.0, 0.0)

        derivative_error = (proportional_error - prev_proportional_error) / self.command_period
        derivative_term = self.kd * derivative_error

        setpoint = proportional_term + integral_term + derivative_term

        command_msg = Point()
        command_msg.x = setpoint.x()
        command_msg.y = setpoint.y()
        command_msg.z = setpoint.z()
        self.get_logger().info(f'Publishing path correction command: [{command_msg.x}, {command_msg.y}, {command_msg.z}] m')
        self.publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Jr3PathCorrection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
