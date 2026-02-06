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

        self.R_jr3_tcp = Rotation.RPY(jr3_roll_param.get_parameter_value().double_value,
                                      jr3_pitch_param.get_parameter_value().double_value,
                                      jr3_yaw_param.get_parameter_value().double_value)

        self.jr3_deadband_forces = jr3_deadband_forces_param.get_parameter_value().double_value
        self.jr3_deadband_torques = jr3_deadband_torques_param.get_parameter_value().double_value

        self.publisher = self.create_publisher(Wrench, 'jr3', 10)

        self.jr3 = Jr3Manager(channel=channel_param.get_parameter_value().string_value,
                              baudrate=baudrate_param.get_parameter_value().integer_value)

        ret, fs, state = self.jr3.get_fs()

        self.get_logger().info(f'JR3 sensor state: {state}, full scale: {fs}')

        self.jr3.stop() # might be already running, ensure it's stopped

        self.get_logger().info('Starting JR3 sensor...')

        self.jr3.start(cutoff_freq=cutoff_param.get_parameter_value().double_value,
                       period_ms=read_period_param.get_parameter_value().double_value)

        success, forces, torques, _ = self.jr3.read()
        self.get_logger().info(f'Initial read success: {success}, forces: {forces}, torques: {torques}')

        self.get_logger().info('Zeroing JR3 sensor...')

        time.sleep(1)
        self.jr3.zero_offs()

        self.get_logger().info('JR3 sensor is running.')

        self.timer = self.create_timer(publish_period_param.get_parameter_value().double_value * 0.001,
                                       self.timer_callback)

    def timer_callback(self):
        success, forces, torques, _ = self.jr3.read()

        if success:
            kdl_forces = Vector(forces[0], forces[1], forces[2])
            kdl_torques = Vector(torques[0], torques[1], torques[2])

            if kdl_forces.Norm() < self.jr3_deadband_forces:
                kdl_forces = Vector.Zero()

            if kdl_torques.Norm() < self.jr3_deadband_torques:
                kdl_torques = Vector.Zero()

            kdl_forces = self.R_jr3_tcp * kdl_forces
            kdl_torques = self.R_jr3_tcp * kdl_torques

            msg = Wrench()
            msg.force = Vector3(x=kdl_forces[0],
                                y=kdl_forces[1],
                                z=kdl_forces[2])
            msg.torque = Vector3(x=kdl_torques[0],
                                 y=kdl_torques[1],
                                 z=kdl_torques[2])
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    jr3_driver = Jr3Driver()
    rclpy.spin(jr3_driver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
