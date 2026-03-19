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

class Jr3AdmittanceController(Node):

    def __init__(self):
        super().__init__('jr3_admittance_controller')
        self.get_logger().info('Starting JR3 admittance controller.')

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

        self.deadband_forces = deadband_forces_param.get_parameter_value().double_value
        self.deadband_torques = deadband_torques_param.get_parameter_value().double_value

        trans_jr3_tcp = kdl.Vector(jr3_trans_x_param.get_parameter_value().double_value,
                                   jr3_trans_y_param.get_parameter_value().double_value,
                                   jr3_trans_z_param.get_parameter_value().double_value)

        R_jr3_tcp = kdl.Rotation.RPY(jr3_roll_param.get_parameter_value().double_value,
                                     jr3_pitch_param.get_parameter_value().double_value,
                                     jr3_yaw_param.get_parameter_value().double_value)

        self.H_jr3_tcp = kdl.Frame(R_jr3_tcp, trans_jr3_tcp)
        self.H_tcp_jr3 = self.H_jr3_tcp.Inverse()

        self.toolWeight_0 = kdl.Wrench() # initializes .wrench to zero
        self.toolWeight_0.force = kdl.Vector(0.0, 0.0, -tool_mass_param.get_parameter_value().double_value * gravity_param.get_parameter_value().double_value)

        self.toolCoM_jr3 = kdl.Vector(tool_com_x_param.get_parameter_value().double_value,
                                      tool_com_y_param.get_parameter_value().double_value,
                                      tool_com_z_param.get_parameter_value().double_value)

        self.wrench_jr3 = None
        self.wrench_jr3_initial = None

        self.jr3_subscription = self.create_subscription(Wrench, 'jr3', self.jr3_listener_callback, 10)
        self.jr3_subscription # prevent unused variable warning

        self.running = True
        self.egm_thread = threading.Thread(target=self.egm_command_worker)
        self.egm_thread.start()

        self.get_logger().info('JR3 admittance controller is running.')

    def jr3_listener_callback(self, msg: Wrench):
        force = kdl.Vector(msg.force.x, msg.force.y, msg.force.z)
        torque = kdl.Vector(msg.torque.x, msg.torque.y, msg.torque.z)
        self.wrench_jr3 = kdl.Wrench(force, torque)

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

                if success and state is not None and state.cartesian is not None and self.wrench_jr3 is not None:
                    R_0_tcp = kdl.Rotation.Quaternion(state.cartesian.orient.u1, state.cartesian.orient.u2, state.cartesian.orient.u3, state.cartesian.orient.u0)
                    p_0 = kdl.Vector(state.cartesian.pos.x, state.cartesian.pos.y, state.cartesian.pos.z)
                    H_0_tcp = kdl.Frame(R_0_tcp, p_0)

                    toolWeight_jr3 = H_0_tcp.M.Inverse() * self.toolWeight_0 # tool weight measured on the CoM
                    toolWeight_jr3 = toolWeight_jr3.RefPoint(-self.toolCoM_jr3) # tool weight measured on the sensor plate

                    wrench_jr3 = self.wrench_jr3 - toolWeight_jr3

                    if self.wrench_jr3_initial is None:
                        self.wrench_jr3_initial = wrench_jr3

                    wrench_jr3 -= self.wrench_jr3_initial

                    if wrench_jr3.force.Norm() < self.deadband_forces:
                        wrench_jr3.force = kdl.Vector.Zero()

                    if wrench_jr3.torque.Norm() < self.deadband_torques:
                        wrench_jr3.torque = kdl.Vector.Zero()

                    wrench_0 = H_0_tcp * (self.H_tcp_jr3 * wrench_jr3)

                    pos = np.array([])
                    orient = np.array([])

                    egm.send_to_robot_cart(state.cartesian.pos, state.cartesian.orient)

                time.sleep(EGM_PERIOD)

def main(args=None):
    rclpy.init(args=args)
    node = Jr3AdmittanceController()

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
