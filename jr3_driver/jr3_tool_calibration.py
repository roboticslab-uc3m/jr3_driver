import rclpy
import math
import time

from .Jr3BaseNode import Jr3BaseNode
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

class Jr3ToolCalibration(Jr3BaseNode):
    def __init__(self):
        super().__init__('jr3_tool_calibration')
        self.get_logger().info('Starting JR3 tool calibration.')

        axis_index_param = self.declare_parameter('axis_index', 0,
            ParameterDescriptor(description='index of the joint axis to calibrate (1-7)', read_only=True))
        displacement_param = self.declare_parameter('displacement', 0.0,
            ParameterDescriptor(description='angular displacement to be applied (deg)', read_only=True))
        duration_param = self.declare_parameter('duration', 0.0,
            ParameterDescriptor(description='duration of the joint movement during half cycle (s)', read_only=True))

        self.axis_index = axis_index_param.get_parameter_value().integer_value
        self.displacement = displacement_param.get_parameter_value().double_value
        self.duration = duration_param.get_parameter_value().double_value

        if self.axis_index < 1 or self.axis_index > 7:
            self.get_logger().error(f'Invalid axis_index: {self.axis_index}. Must be between 1 and 7.')
            raise ValueError(f'Invalid axis_index: {self.axis_index}. Must be between 1 and 7.')
        else:
            self.get_logger().info(f'Using axis_index: {self.axis_index}')

        if self.displacement == 0:
            self.get_logger().error(f'Invalid displacement: {self.displacement}. Must be non-zero.')
            raise ValueError(f'Invalid displacement: {self.displacement}. Must be non-zero.')
        else:
            self.get_logger().info(f'Using displacement: {self.displacement}')

        if self.duration <= 0:
            self.get_logger().error(f'Invalid duration: {self.duration}. Must be positive.')
            raise ValueError(f'Invalid duration: {self.duration}. Must be positive.')
        else:
            self.get_logger().info(f'Using duration: {self.duration}')

        self.initial_value = None
        self.current_value = None
        self.step = 0
        self.start_time = None

        self.joint_state_subscription = self.create_subscription(JointState, 'state/joint', self.joint_state_callback, 10)
        self.joint_state_subscription # prevent unused variable warning

        while self.initial_value is None:
            self.get_logger().info('Waiting for initial joint state...')
            rclpy.spin_once(self, timeout_sec=1.0)
            time.sleep(0.1)

        self.publisher = self.create_publisher(Float32MultiArray, 'command/joint', 10)
        self.get_logger().info('JR3 tool calibration is running.')

    def joint_state_callback(self, msg: JointState):
        if self.initial_value is None:
            self.initial_value = list(map(math.degrees, msg.position))
            self.current_value = list(self.initial_value)

    def send_command(self, wrench_0, H_0_tcp):
        if self.current_value is None or self.initial_value is None:
            return

        if self.start_time is None:
            self.start_time = time.monotonic()

        # Sinusoidal velocity profile with half-cycle duration `self.duration`.
        # The position oscillates between q0 and q0 + displacement.
        elapsed = time.monotonic() - self.start_time
        full_cycle = 2.0 * self.duration
        cycle_time = elapsed % full_cycle

        if cycle_time < self.duration:
            # Outbound half-cycle: q0 -> q0 + displacement
            phase = cycle_time
            delta = 0.5 * self.displacement * (1.0 - math.cos(math.pi * phase / self.duration))
        else:
            # Return half-cycle: q0 + displacement -> q0
            phase = cycle_time - self.duration
            delta = self.displacement - 0.5 * self.displacement * (1.0 - math.cos(math.pi * phase / self.duration))

        axis = self.axis_index - 1
        self.current_value[axis] = self.initial_value[axis] + delta

        msg = Float32MultiArray()
        msg.data = list(map(math.radians, self.current_value))
        self.publisher.publish(msg)

        self.step += 1
        # self.get_logger().info(f'Wrench: {wrench_0}')

def main(args=None):
    rclpy.init(args=args)
    node = Jr3ToolCalibration()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
