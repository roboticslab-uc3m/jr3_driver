import numpy as np
import PyKDL as kdl
import rclpy
import threading
import time

from rclpy.node import Node
from geometry_msgs.msg import Wrench, Vector3
from rcl_interfaces.msg import ParameterDescriptor
from ABBRobotEGM import EGM

EGM_PERIOD = 0.004 # seconds, 250 Hz

class Jr3ImpedanceController(Node):

    def __init__(self):
        super().__init__('jr3_impedance_controller')
        self.get_logger().info('Starting JR3 impedance controller.')

        deadband_forces_param = self.declare_parameter('deadband_forces', 0.0,
            ParameterDescriptor(description='deadband on force measurements (N)'))
        deadband_torques_param = self.declare_parameter('deadband_torques', 0.0,
            ParameterDescriptor(description='deadband on torque measurements (N*m)'))
        jr3_roll_param = self.declare_parameter('jr3_roll', 0.0,
            ParameterDescriptor(description='sensor frame roll (rad)'))
        jr3_pitch_param = self.declare_parameter('jr3_pitch', 0.0,
            ParameterDescriptor(description='sensor frame pitch (rad)'))
        jr3_yaw_param = self.declare_parameter('jr3_yaw', 0.0,
            ParameterDescriptor(description='sensor frame yaw (rad)'))
        gravity_param = self.declare_parameter('gravity', 9.8,
            ParameterDescriptor(description='gravity acceleration (m/s^2)'))
        tool_mass_param = self.declare_parameter('tool_mass', 0.0,
            ParameterDescriptor(description='mass of the tool attached to the sensor (kg)'))
        tool_com_x_param = self.declare_parameter('tool_com_x', 0.0,
            ParameterDescriptor(description='x coordinate of the tool center of mass in the sensor frame (m)'))
        tool_com_y_param = self.declare_parameter('tool_com_y', 0.0,
            ParameterDescriptor(description='y coordinate of the tool center of mass in the sensor frame (m)'))
        tool_com_z_param = self.declare_parameter('tool_com_z', 0.0,
            ParameterDescriptor(description='z coordinate of the tool center of mass in the sensor frame (m)'))
        linear_stiffness_param = self.declare_parameter('linear_stiffness', 0.10,
            ParameterDescriptor(description='impedance controller linear stiffness (N/m)'))
        angular_stiffness_param = self.declare_parameter('angular_stiffness', 0.1,
            ParameterDescriptor(description='impedance controller angular stiffness (N*m/rad)'))
        linear_damping_param = self.declare_parameter('linear_damping', 0.1,
            ParameterDescriptor(description='impedance controller linear damping (N*s/m)'))
        angular_damping_param = self.declare_parameter('angular_damping', 0.1,
            ParameterDescriptor(description='impedance controller angular damping (N*m*s/rad)'))

        self.deadband_forces = deadband_forces_param.get_parameter_value().double_value
        self.deadband_torques = deadband_torques_param.get_parameter_value().double_value

        self.linear_stiffness = linear_stiffness_param.get_parameter_value().double_value
        self.angular_stiffness = angular_stiffness_param.get_parameter_value().double_value
        self.linear_damping = linear_damping_param.get_parameter_value().double_value
        self.angular_damping = angular_damping_param.get_parameter_value().double_value

        self.R_jr3_tcp = kdl.Rotation.RPY(jr3_roll_param.get_parameter_value().double_value,
                                          jr3_pitch_param.get_parameter_value().double_value,
                                          jr3_yaw_param.get_parameter_value().double_value)

        self.toolWeight_0 = kdl.Vector(0.0, 0.0, -tool_mass_param.get_parameter_value().double_value * gravity_param.get_parameter_value().double_value)

        self.toolCoM_N = kdl.Vector(tool_com_x_param.get_parameter_value().double_value,
                                    tool_com_y_param.get_parameter_value().double_value,
                                    tool_com_z_param.get_parameter_value().double_value)

        self.wrench = None
        self.wrench_initial = None

        self.jr3_subscription = self.create_subscription(Wrench, 'jr3', self.jr3_listener_callback, 10)
        self.jr3_subscription # prevent unused variable warning

        self.running = True
        self.egm_thread = threading.Thread(target=self.egm_command_worker)
        self.egm_thread.start()

        self.get_logger().info('JR3 impedance controller is running.')

    def jr3_listener_callback(self, msg: Wrench):
        forces = kdl.Vector(msg.force.x, msg.force.y, msg.force.z)
        torques = kdl.Vector(msg.torque.x, msg.torque.y, msg.torque.z)
        self.wrench = kdl.Wrench(forces, torques)

    def egm_command_worker(self):
        with EGM() as egm:
            self.get_logger().info('EGM command worker started, waiting for connection from robot.')

            while self.running:
                success, _ = egm.receive_from_robot(timeout=1.0)

                if success:
                    self.get_logger().info('EGM connection established.')
                    break

            while self.running:
                success, state = egm.receive_from_robot(timeout=0.01)

                if success and self.wrench is not None:
                    R_0_N = kdl.Rotation.Quaternion(state.cartesian.orient.u1, state.cartesian.orient.u2, state.cartesian.orient.u3, state.cartesian.orient.u0)
                    p_0 = kdl.Vector(state.cartesian.pos.x, state.cartesian.pos.y, state.cartesian.pos.z)
                    H_0_N = kdl.Frame(R_0_N, p_0)

                    toolWrench = H_0_N.Inverse() * self.toolWeight_0
                    toolWrench_N = toolWrench.RefPoint(self.toolCoM_N)

                    wrench = self.wrench - toolWrench_N

                    if self.wrench_initial is None:
                        self.wrench_initial = wrench

                    wrench -= self.wrench_initial

                    if wrench.force.Norm() < self.deadband_forces:
                        wrench.force = kdl.Vector.Zero()

                    if wrench.torque.Norm() < self.deadband_torques:
                        wrench.torque = kdl.Vector.Zero()

                    pos = np.array([])
                    orient = np.array([])

                    egm.send_to_robot_cart(pos, orient)

                time.sleep(EGM_PERIOD)

def main(args=None):
    rclpy.init(args=args)
    node = Jr3ImpedanceController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.egm_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
