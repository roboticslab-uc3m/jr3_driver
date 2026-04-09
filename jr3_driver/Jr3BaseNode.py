import PyKDL as kdl

from rclpy.node import Node
from geometry_msgs.msg import Pose, Wrench
from rcl_interfaces.msg import ParameterDescriptor
from abc import ABC, abstractmethod

COMMAND_PERIOD = 0.01 # seconds, 100 Hz
GRAVITY = 9.8 # m/s^2

class Jr3BaseNode(ABC, Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        command_period_param = self.declare_parameter('command_period', COMMAND_PERIOD,
            ParameterDescriptor(description='Command update period (s)'))
        deadband_forces_param = self.declare_parameter('deadband_forces', 0.0,
            ParameterDescriptor(description='deadband on force measurements (N)'))
        deadband_torques_param = self.declare_parameter('deadband_torques', 0.0,
            ParameterDescriptor(description='deadband on torque measurements (N*m)'))
        jr3_trans_x_param = self.declare_parameter('jr3_trans_x', 0.0,
            ParameterDescriptor(description='x translation from sensor frame to tool frame (m)'))
        jr3_trans_y_param = self.declare_parameter('jr3_trans_y', 0.0,
            ParameterDescriptor(description='y translation from sensor frame to tool frame (m)'))
        jr3_trans_z_param = self.declare_parameter('jr3_trans_z', 0.0,
            ParameterDescriptor(description='z translation from sensor frame to tool frame (m)'))
        jr3_roll_param = self.declare_parameter('jr3_roll', 0.0,
            ParameterDescriptor(description='tool frame roll (rad) wrt. sensor frame'))
        jr3_pitch_param = self.declare_parameter('jr3_pitch', 0.0,
            ParameterDescriptor(description='tool frame pitch (rad) wrt. sensor frame'))
        jr3_yaw_param = self.declare_parameter('jr3_yaw', 0.0,
            ParameterDescriptor(description='tool frame yaw (rad) wrt. sensor frame'))
        gravity_param = self.declare_parameter('gravity', GRAVITY,
            ParameterDescriptor(description='gravity acceleration (m/s^2)'))
        tool_mass_param = self.declare_parameter('tool_mass', 0.0,
            ParameterDescriptor(description='mass of the tool attached to the sensor (kg)'))
        tool_com_x_param = self.declare_parameter('tool_com_x', 0.0,
            ParameterDescriptor(description='x coordinate of the tool center of mass in the sensor frame (m)'))
        tool_com_y_param = self.declare_parameter('tool_com_y', 0.0,
            ParameterDescriptor(description='y coordinate of the tool center of mass in the sensor frame (m)'))
        tool_com_z_param = self.declare_parameter('tool_com_z', 0.0,
            ParameterDescriptor(description='z coordinate of the tool center of mass in the sensor frame (m)'))

        self.command_period = command_period_param.get_parameter_value().double_value

        if self.command_period <= 0.0:
            self.get_logger().error(f'Invalid command_period: {self.command_period}. Must be greater than 0.0.')
            raise ValueError(f'Invalid command_period: {self.command_period}. Must be greater than 0.0.')
        else:
            self.get_logger().info(f'Using command_period: {self.command_period}')

        self.deadband_forces = deadband_forces_param.get_parameter_value().double_value
        self.deadband_torques = deadband_torques_param.get_parameter_value().double_value

        if self.deadband_forces < 0.0:
            self.get_logger().error(f'Invalid deadband_forces: {self.deadband_forces}. Must be greater than or equal to 0.0.')
            raise ValueError(f'Invalid deadband_forces: {self.deadband_forces}. Must be greater than or equal to 0.0.')

        if self.deadband_torques < 0.0:
            self.get_logger().error(f'Invalid deadband_torques: {self.deadband_torques}. Must be greater than or equal to 0.0.')
            raise ValueError(f'Invalid deadband_torques: {self.deadband_torques}. Must be greater than or equal to 0.0.')

        self.get_logger().info(f'Using deadband_forces: {self.deadband_forces}')
        self.get_logger().info(f'Using deadband_torques: {self.deadband_torques}')

        trans_jr3_tcp = kdl.Vector(jr3_trans_x_param.get_parameter_value().double_value,
                                   jr3_trans_y_param.get_parameter_value().double_value,
                                   jr3_trans_z_param.get_parameter_value().double_value)

        R_jr3_tcp = kdl.Rotation.RPY(jr3_roll_param.get_parameter_value().double_value,
                                     jr3_pitch_param.get_parameter_value().double_value,
                                     jr3_yaw_param.get_parameter_value().double_value)

        self.H_jr3_tcp = kdl.Frame(R_jr3_tcp, trans_jr3_tcp)
        self.H_tcp_jr3 = self.H_jr3_tcp.Inverse()

        gravity = gravity_param.get_parameter_value().double_value
        tool_mass = tool_mass_param.get_parameter_value().double_value

        if tool_mass < 0.0:
            self.get_logger().error(f'Invalid tool_mass: {tool_mass}. Must be greater than 0.0.')
            raise ValueError(f'Invalid tool_mass: {tool_mass}. Must be greater than 0.0.')
        else:
            self.get_logger().info(f'Using tool_mass: {tool_mass}')

        self.toolWeight_0 = kdl.Wrench() # initializes .wrench to zero
        self.toolWeight_0.force = kdl.Vector(0.0, 0.0, -tool_mass * gravity)

        self.toolCoM_jr3 = kdl.Vector(tool_com_x_param.get_parameter_value().double_value,
                                      tool_com_y_param.get_parameter_value().double_value,
                                      tool_com_z_param.get_parameter_value().double_value)

        self.wrench_tcp = None
        self.wrench_tcp_initial = None

        self.jr3_subscription = self.create_subscription(Wrench, 'jr3', self.jr3_listener_callback, 10)
        self.jr3_subscription # prevent unused variable warning

        self.current_pose = None

        self.egm_subscription = self.create_subscription(Pose, 'state/pose', self.egm_state_callback, 10)
        self.egm_subscription # prevent unused variable warning

        self.command_thread = self.create_timer(self.command_period, self.command_worker)

    def jr3_listener_callback(self, msg: Wrench):
        force = kdl.Vector(msg.force.x, msg.force.y, msg.force.z)
        torque = kdl.Vector(msg.torque.x, msg.torque.y, msg.torque.z)
        self.wrench_tcp = self.H_tcp_jr3 * kdl.Wrench(force, torque)

    def egm_state_callback(self, msg: Pose):
        p = kdl.Vector(msg.position.x, msg.position.y, msg.position.z)
        q = kdl.Rotation.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.current_pose = kdl.Frame(q, p)

    def command_worker(self):
        if self.current_pose is not None and self.wrench_tcp is not None:
            H_0_tcp = kdl.Frame(self.current_pose)

            toolWeight_tcp = H_0_tcp.M.Inverse() * self.toolWeight_0 # tool weight measured on the CoM
            toolWeight_tcp = toolWeight_tcp.RefPoint(self.H_jr3_tcp.p - self.toolCoM_jr3) # tool weight measured on the TCP

            wrench_tcp = self.wrench_tcp - toolWeight_tcp

            if self.wrench_tcp_initial is None:
                self.wrench_tcp_initial = kdl.Wrench(wrench_tcp)

            wrench_tcp -= self.wrench_tcp_initial

            if wrench_tcp.force.Norm() < self.deadband_forces:
                wrench_tcp.force = kdl.Vector.Zero()

            if wrench_tcp.torque.Norm() < self.deadband_torques:
                wrench_tcp.torque = kdl.Vector.Zero()

            wrench_0 = H_0_tcp.M * wrench_tcp

            self.send_command(wrench_0, self.wrench_tcp, H_0_tcp)

    @abstractmethod
    def send_command(self, wrench_0, wrench_tcp, H_0_tcp):
        pass
