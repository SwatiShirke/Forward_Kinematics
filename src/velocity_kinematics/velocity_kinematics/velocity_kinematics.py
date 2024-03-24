import math
import numpy as np

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node

# from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose,Point,Quaternion
from interfaces.srv import GiveEefVelocity, GiveJointVelocity
from sensor_msgs.msg import JointState

from math import sqrt, pi, cos, sin


class VelocityKinematics(Node):
    qos = QoSProfile(depth=10)

    def __init__(self):
        super().__init__('velocity_kinematics')
        self.get_logger().info("Starting Velocity Kinematics Node")

        self.srv = self.create_service(GiveEefVelocity, 'give_eef_velocities', self.give_eef_velocities)
        self.srv = self.create_service(GiveJointVelocity, 'give_joint_velocities', self.give_joint_velocities)

        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, self.qos)
        self.measured_joint_values = []

    def joint_state_callback(self, msg):
        self.measured_joint_values = msg.position[:-1]

    def calculate_matrix(self):
        l0 = 36.076
        l1 = 96.326-36.076
        l2 = sqrt(128**2 + 24**2)
        l3 = 124
        l4 = 133.4

        bug_fix = math.atan2(128, 24)

        q1,q2,q3,q4 = self.measured_joint_values
        self.get_logger().info(f'q values: {[q1,q2,q3,q4]}') 
        self.get_logger().info(f'l values: {[l1,l2,l3,l4]}') 
        A1 = self.calc_dh_trans(0,-pi/2,q1,l1)
        A2 = self.calc_dh_trans(l2,0,q2-bug_fix,0)
        A3 = self.calc_dh_trans(l3,0,q3+bug_fix,0)
        A4 = self.calc_dh_trans(l4,0,q4,0)

        r = np.round
        self.get_logger().info(f"A1\n{r(A1)}")
        self.get_logger().info(f"A2\n{r(A2)}")
        self.get_logger().info(f"A3\n{r(A3)}")
        self.get_logger().info(f"A4\n{r(A4)}")


        T01 = A1
        T02 = np.dot(A1, A2)
        T03 = np.dot(T02 , A3)
        T04 = np.dot(T03 , A4)

        o0 = np.array([0, 0, 0])
        o1 = T01[0:3,3]
        o2 = T02[0:3,3]
        o3 = T03[0:3,3]
        o4 = T04[0:3,3]
        
        self.get_logger().info(f"{o1}")
        self.get_logger().info(f"{o2}")
        self.get_logger().info(f"{o3}")
        self.get_logger().info(f"{o4}")

        z = np.array([0,0,1])
        z0 = [0, 0, 1]
        z1 = T01[0:3,0:3] @ z
        z2 = T02[0:3,0:3] @ z
        z3 = T03[0:3,0:3] @ z
        z4 = T04[0:3,0:3] @ z
        
        J1 = [*np.cross(z0,o4-o0), *z0]
        J2 = [*np.cross(z1,o4-o1), *z1]
        J3 = [*np.cross(z2,o4-o2), *z2]
        J4 = [*np.cross(z3,o4-o3), *z3]

        self.get_logger().info(f'j1 {r(J1)}')
        self.get_logger().info(f'j2 {r(J2)}')
        self.get_logger().info(f'j3 {r(J3)}')
        self.get_logger().info(f'j4 {r(J4)}')

        self.J = np.array([J1,J2,J3,J4]).T
        self.J_inv = np.linalg.pinv(self.J)
        self.get_logger().info(f'j\n {np.round(self.J,2)}')
        self.get_logger().info(f'j inv\n {np.round(self.J_inv,2)}')

    def give_eef_velocities(self, request, response):
        #extract joint velocities from input JointState msg
        self.get_logger().info("Got Request for End Effector Velocity")

        theta_dot = np.array(request.joint_velocities)

        self.calculate_matrix()
        response.eef_velocity = self.J @ theta_dot
        self.get_logger().info("Sent Response")
        
        return response
    
    def give_joint_velocities(self, request, response):
        #extract joint velocities from input JointState msg
        self.get_logger().info("Got Request for End Effector Velocity")
        self.eef_velocities = request.eef_velocities

        #create 6x1 array of joint velocities
        eef_velocity = np.array(self.eef_velocities)

        self.calculate_matrix()
        response.joint_velocities = self.J_inv @ eef_velocity
        self.get_logger().info("Sent Response")
        
        return response

    def calc_dh_trans(self, a, alpha, theta, d):
            T = np.array([
                    [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                    [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                    [0, sin(alpha), cos(alpha), d],
                    [0, 0, 0, 1]
            ])
            #round output to 2 decimal places 
            return T

# Initialize ros and start spinning the node   
def main(args=None):
    rclpy.init(args=args)
    node = VelocityKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()