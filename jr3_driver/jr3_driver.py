import rclpy
import time

from rclpy.node import Node
from geometry_msgs.msg import Wrench, Vector3
from rcl_interfaces.msg import ParameterDescriptor
from PyKDL import Rotation, Vector
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
        jr3_roll_param = self.declare_parameter('jr3_roll', 0.0,
            ParameterDescriptor(description='JR3 frame roll (rad)'))
        jr3_pitch_param = self.declare_parameter('jr3_pitch', 0.0,
            ParameterDescriptor(description='JR3 frame pitch (rad)'))
        jr3_yaw_param = self.declare_parameter('jr3_yaw', 0.0,
            ParameterDescriptor(description='JR3 frame yaw (rad)'))
        jr3_deadband_forces_param = self.declare_parameter('jr3_deadband_forces', 0.0,
            ParameterDescriptor(description='JR3 deadband on force measurements (N)'))
        jr3_deadband_torques_param = self.declare_parameter('jr3_deadband_torques', 0.0,
            ParameterDescriptor(description='JR3 deadband on torque measurements (N*m)'))

        self._R_jr3_tcp = Rotation.RPY(jr3_roll_param.get_parameter_value().double_value,
                                      jr3_pitch_param.get_parameter_value().double_value,
                                      jr3_yaw_param.get_parameter_value().double_value)

        self._jr3_deadband_forces = jr3_deadband_forces_param.get_parameter_value().double_value
        self._jr3_deadband_torques = jr3_deadband_torques_param.get_parameter_value().double_value

        self._publisher = self.create_publisher(Wrench, 'jr3', 10)

        self.jr3 = Jr3Manager(channel=channel_param.get_parameter_value().string_value,
                              baudrate=baudrate_param.get_parameter_value().integer_value)

        self._timer = None

        try:
            self._setup_jr3(cutoff_param, read_period_param)

            self._timer = self.create_timer(publish_period_param.get_parameter_value().double_value * 0.001,
                                            self._timer_callback)

            self.get_logger().info('JR3 sensor is running.')
        except Exception as e:
            self.get_logger().fatal(f'Initialization failed: {e}')
            self.close()
            raise e

    def _setup_jr3(self, cutoff_param, read_period_param):
        ret, fs, state = self.jr3.get_fs()

        if not ret or state != self.jr3._state.READY:
            raise RuntimeError('Failed to get JR3 sensor full scale factors or sensor not ready')

        self.get_logger().info(f'JR3 sensor state: {state}, full scale: {fs}')
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

        if self._timer is not None:
            self._timer.cancel()

        if hasattr(self, 'jr3') and self.jr3 is not None:
            self.jr3.close()

    def _timer_callback(self):
        success, forces, torques, _ = self.jr3.read()

        if success:
            kdl_forces = Vector(forces[0], forces[1], forces[2])
            kdl_torques = Vector(torques[0], torques[1], torques[2])

            if kdl_forces.Norm() < self._jr3_deadband_forces:
                kdl_forces = Vector.Zero()

            if kdl_torques.Norm() < self._jr3_deadband_torques:
                kdl_torques = Vector.Zero()

            kdl_forces = self._R_jr3_tcp * kdl_forces
            kdl_torques = self._R_jr3_tcp * kdl_torques

            msg = Wrench()
            msg.force = Vector3(x=kdl_forces[0],
                                y=kdl_forces[1],
                                z=kdl_forces[2])
            msg.torque = Vector3(x=kdl_torques[0],
                                 y=kdl_torques[1],
                                 z=kdl_torques[2])
            self._publisher.publish(msg)

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
