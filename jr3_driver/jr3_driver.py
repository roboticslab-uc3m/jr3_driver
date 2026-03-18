import rclpy
import time

from rclpy.node import Node
from geometry_msgs.msg import Wrench, Vector3
from rcl_interfaces.msg import ParameterDescriptor
from .Jr3Manager import Jr3Manager

class Jr3Driver(Node):

    def __init__(self):
        super().__init__('jr3_driver')

        self.context.on_shutdown(self.close)

        channel_param = self.declare_parameter('channel', '/dev/ttyLPC1768',
            ParameterDescriptor(description='Serial channel name'))
        baudrate_param = self.declare_parameter('baudrate', 115200,
            ParameterDescriptor(description='Serial baudrate (bps)'))
        cutoff_param = self.declare_parameter('cutoff_frequency', 2.0,
            ParameterDescriptor(description='Cutoff frequency (Hz)'))
        read_period_param = self.declare_parameter('read_period', 10.0,
            ParameterDescriptor(description='Sensor read period (ms)'))
        publish_period_param = self.declare_parameter('publish_period', 20.0,
            ParameterDescriptor(description='Publish period (ms)'))

        self.publisher = self.create_publisher(Wrench, 'jr3', 10)

        self.jr3 = Jr3Manager(channel=channel_param.get_parameter_value().string_value,
                              baudrate=baudrate_param.get_parameter_value().integer_value)

        self.timer = None

        try:
            self.setup_jr3(cutoff_param, read_period_param)

            self.timer = self.create_timer(publish_period_param.get_parameter_value().double_value * 0.001,
                                           self.timer_callback)

            self.get_logger().info('JR3 sensor is running.')
        except Exception as e:
            self.get_logger().fatal(f'Initialization failed: {e}')
            self.close()
            raise e

    def setup_jr3(self, cutoff_param, read_period_param):
        ret, fs, state = self.jr3.get_fs()

        if not ret or state != self.jr3._state.READY:
            raise RuntimeError('Failed to get JR3 sensor full scale factors or sensor not ready')

        self.get_logger().info(f'JR3 sensor state: {state}, full scales: {fs}')
        self.jr3.stop() # might be already running, so ensure it's stopped
        self.get_logger().info('Starting JR3 sensor...')

        ret, state = self.jr3.start(cutoff_freq=cutoff_param.get_parameter_value().double_value,
                                    period_ms=read_period_param.get_parameter_value().double_value)

        if not ret:
            raise RuntimeError('Failed to start JR3 sensor')

        self.get_logger().info('Zeroing JR3 sensor...')
        time.sleep(1)
        ret, state = self.jr3.zero_offs()

        if not ret:
            raise RuntimeError('Failed to zero JR3 sensor')

    def close(self):
        self.get_logger().info('Closing JR3 driver.')

        if self.timer is not None:
            self.timer.cancel()

        if hasattr(self, 'jr3') and self.jr3 is not None:
            self.jr3.close()

    def timer_callback(self):
        success, forces, torques, _ = self.jr3.read()

        if success:
            msg = Wrench()
            msg.force = Vector3(x=forces[0], y=forces[1], z=forces[2])
            msg.torque = Vector3(x=torques[0], y=torques[1], z=torques[2])
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    try:
        driver = Jr3Driver()

        try:
            rclpy.spin(driver)
        except KeyboardInterrupt:
            pass
        finally:
            driver.close()
            driver.destroy_node()
    except RuntimeError:
        pass
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
