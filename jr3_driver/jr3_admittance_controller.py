import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Wrench, Vector3
from rcl_interfaces.msg import ParameterDescriptor
from PyKDL import Rotation, Vector
from ABBRobotEGM import EGM

class Jr3AdmittanceController(Node):

    def __init__(self):
        super().__init__('jr3_admittance_controller')
        self.get_logger().info('Starting JR3 admittance controller.')

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

        self.deadband_forces = deadband_forces_param.get_parameter_value().double_value
        self.deadband_torques = deadband_torques_param.get_parameter_value().double_value

        self.R_jr3_tcp = Rotation.RPY(jr3_roll_param.get_parameter_value().double_value,
                                       jr3_pitch_param.get_parameter_value().double_value,
                                       jr3_yaw_param.get_parameter_value().double_value)

        self.toolWeight_0 = Vector(0.0, 0.0, -tool_mass_param.get_parameter_value().double_value * gravity_param.get_parameter_value().double_value)
        self.toolCoM_N = Vector(tool_com_x_param.get_parameter_value().double_value,
                                tool_com_y_param.get_parameter_value().double_value,
                                tool_com_z_param.get_parameter_value().double_value)

        self.jr3_subscription = self.create_subscription(Wrench, 'jr3', self.jr3_listener_callback, 10)
        self.jr3_subscription # prevent unused variable warning

        self.timer = self.create_timer(0.048, self.egm_timer_callback)
        self.timer # prevent unused variable warning

        self.get_logger().info('JR3 admittance controller is running.')

    def jr3_listener_callback(self, msg: Wrench):
        forces = Vector(msg.force.x, msg.force.y, msg.force.z)
        torques = Vector(msg.torque.x, msg.torque.y, msg.torque.z)

        if forces.Norm() < self.deadband_forces:
            forces = Vector.Zero()

        if torques.Norm() < self.deadband_torques:
            torques = Vector.Zero()

    def egm_timer_callback(self):
        pass

def main(args=None):
    rclpy.init(args=args)

    try:
        driver = Jr3AdmittanceController()

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
