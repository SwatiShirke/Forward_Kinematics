import math
import numpy as np
import time
import csv

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node

# from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose,Point,Quaternion
from open_manipulator_msgs.msg import KinematicsPose 
from sensor_msgs.msg import JointState

from transforms3d import quaternions

class ForwardKinematics(Node):
    qos = QoSProfile(depth=10)
    
    def __init__(self):
        super().__init__('forward_kinematics')
        self.get_logger().info("Starting the Forward Kinematics Node")

        self.current_joint_values = [0,0,0,0]
        
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, self.qos)

        self.calculated_pose = self.create_publisher(Pose, 'calculated_pose', 10)

        self.csv_file_path = 'log_file.csv'
        self.csv_file = open(self.csv_file_path,'w',newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # For verifying pose with the Robot provided pose
        # self.current_pose = KinematicsPose() 
        # self.create_subscription(KinematicsPose, 'current_pose', self.kinematics_pose_callback, 10)
            
    def joint_state_callback(self, msg):
            positions = msg.position
            self.current_joint_values = positions[:-1]
            orient, position = self.calculate_forward_kinematics(self.current_joint_values)

            msg = Pose()

            msg.position = Point()
            msg.position.x = position[0]
            msg.position.y = position[1]
            msg.position.z = position[2]

            msg.orientation = Quaternion()
            # msg.orientation.
            quat = quaternions.mat2quat(np.array(orient))
            msg.orientation.w = quat[0]
            msg.orientation.x = quat[1]
            msg.orientation.y = quat[2]
            msg.orientation.z = quat[3]

            self.calculated_pose.publish(msg)
                
            # # For Debugging
            # joint_values = [ round(x, 3) for x in self.current_joint_values]
            # self.get_logger().info(f"positions {joint_values}")

            #timestamp = self.get_clock().now().nanoseconds
            joint_pos = position
            self.csv_writer.writerow(joint_pos)
            
            self.get_logger().info(f"Input joint angles were: {self.current_joint_values} radians")
            self.get_logger().info(f"End effector orient: \n\t{orient[0]}\n\t{orient[1]}\n\t{orient[2]}")
            self.get_logger().info(f"End effector position: \n\t{position}")
            time.sleep(0.25)
            
    
    def kinematics_pose_callback(self, msg):
            self.current_pose = msg
            x = self.current_pose.pose.orientation.x
            y = self.current_pose.pose.orientation.y
            z = self.current_pose.pose.orientation.z
            w = self.current_pose.pose.orientation.w

            pose_from_bot = self.quaternion_rotation_matrix(w,x,y,z)
            position_from_bot = [self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z]

            self.get_logger().info(f"End effector orientation from robot: \n{pose_from_bot}")
            self.get_logger().info(f"End effector position from robot: \n{position_from_bot}")
            self.verify()

    # calculate the transformation matrix for the RRR cylindrical manipulator
    def calculate_forward_kinematics(self, q):
            q1, q2, q3,q4 = q
            l0 = 36.076
            l1 = 96.326-36.076
            l2 = math.sqrt(128**2 + 24**2)
            l3 = 124
            l4 = 133.4

            A1 = self.calc_dh_trans(0,0,0,l0)
            A2 = self.calc_dh_trans(0,-math.pi/2,q1,l1)
            A3 = self.calc_dh_trans(l2,0,q2-math.pi/2,0)
            A4 = self.calc_dh_trans(l3,0,q3+math.pi/2,0)
            A5 = self.calc_dh_trans(l4,0,q4,0)
            T = np.dot(np.dot((np.dot(np.dot(A1,A2),A3)),A4),A5)
            R = T[:3,:3]
            P = T[0:3, 3]
            f = [[round(m, 1) for m in n] for n in R]
            fP = [round(m, 1) for m in P]

            return f, fP
            
    #generic function to calculate the trans matrix of any joint given its DH params
    def calc_dh_trans(self, a, alpha, theta, d):
            T = np.array([
                    [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
                    [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
                    [0, math.sin(alpha), math.cos(alpha), d],
                    [0, 0, 0, 1]
            ])
            #round output to 2 decimal places 
            return np.round(T,2)

    def quaternion_rotation_matrix(self, qw, qx, qy, qz):
            """ Covert a quaternion into a full three-dimensional rotation matrix.

            Input
            :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 

            Output
            :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                    This rotation matrix converts a point in the local reference 
                    frame to a point in the global reference frame.
            """
            # Extract the values from Q
            q0 = qw
            q1 = qx
            q2 = qy
            q3 = qz

            # First row of the rotation matrix
            r00 = round(2 * (q0 * q0 + q1 * q1) - 1, 2)
            r01 = round(2 * (q1 * q2 - q0 * q3), 2)
            r02 = round(2 * (q1 * q3 + q0 * q2), 2)

            # Second row of the rotation matrix
            r10 = round(2 * (q1 * q2 + q0 * q3), 2)
            r11 = round(2 * (q0 * q0 + q2 * q2) - 1, 2)
            r12 = round(2 * (q2 * q3 - q0 * q1), 2)

            # Third row of the rotation matrix
            r20 = round(2 * (q1 * q3 - q0 * q2), 2)
            r21 = round(2 * (q2 * q3 + q0 * q1), 2)
            r22 = round(2 * (q0 * q0 + q3 * q3) - 1, 2)

            # 3x3 rotation matrix
            rot_matrix = np.array([[r00, r01, r02],
                                    [r10, r11, r12],
                                    [r20, r21, r22]])
            return rot_matrix
    

# Initialize ros and start spinning the node   
def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()       


                
