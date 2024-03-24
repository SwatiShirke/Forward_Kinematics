import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
import numpy as np
from pprint import pformat

import tf2_py
import tf2_ros
import math
#testing track changes
#Create class containing calc_rrr_fk_node
from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile


class calc_rrr_fk_node(Node):
        qos = QoSProfile(depth=10)

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

        def joint_state_callback(self, msg):
                positions = msg.position
                self.current_joint_values = positions[:-1]
                # joint_values = [ round(x, 3) for x in self.current_joint_values]
                # self.get_logger().info(f"positions {joint_values}")

        
        def __init__(self):
                super().__init__('calc_rrr_fk_node')
                self.subscription = self.create_subscription(
                        KinematicsPose, #msg type
                        'current_pose', #topic name
                        self.q_callback,
                        10
                )
                
                self.joint_state_subscription = self.create_subscription(
                        JointState,
                        'joint_states',
                        self.joint_state_callback,
                        self.qos
                        )


                #new subscriber for inverse kinematics
                self.subscription = self.create_subscription(Pose, 'ee_pose' ,self.pose_callback, 10)
                self.current_joint_values = [0,0,0,0]
                
        
        def q_callback(self, msg):
                
                # self.get_logger().info(f"End effector\n\tposition: \n{msg.pose.position}\n\t orientation: {msg.pose.orientation}")
                x = msg.pose.orientation.x
                y = msg.pose.orientation.y
                z = msg.pose.orientation.z
                w = msg.pose.orientation.w
                
                pose_from_bot = self.quaternion_rotation_matrix(w,x,y,z)
                position_from_bot = [msg.pose.position.x,  msg.pose.position.y, msg.pose.position.z,]

                # self.get_logger().info(f"Rotation Matrix: {r}")
                
                
                # get joint angle inputs
                # q_input = msg.data

                #send input to FK function
                orient, position = self.calc_rrr_fk(self.current_joint_values)

                #print data to the terminal
                self.get_logger().info(f"Input joint angles were: {self.current_joint_values} radians")
                self.get_logger().info(f"End effector orient: \n\t{orient[0]}\n\t{orient[1]}\n\t{orient[2]}")
                self.get_logger().info(f"End effector position: \n\t{position}")

                self.get_logger().info(f"End effector orientation from robot: \n{pose_from_bot}")
                self.get_logger().info(f"End effector position from robot: \n{position_from_bot}")
        
        #callback function for IK kinematics
        def pose_callback(self, pose_msg):
               
               #get position and orientation from input Pose msg
               position = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
               orientation = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]

               #calculate q values using position and orientation data
               q_values = self.calc_ik(position,orientation)
                #print data to the terminal
               self.get_logger().info(f"Input end effector pose was: {pose_msg}")
               self.get_logger().info(f"Required joint angles to reach pose are {q_values}")
               

        #calculate the transformation matrix for the RRR cylindrical manipulator
        def calc_rrr_fk(self, q):
                #ASSSUMING ALL LINKS HAVE LENGTH = 1, FUNCTION INPUT MAY BE ADJUSTED FOR DIFFERENT LINK LENGTHS
                q1, q2, q3,q4 = q
                l0 = 36.076
                l1 = 96.326-36.076
                l2 = math.sqrt(128**2 + 24**2)
                l3 = 124
                l4 = 133.4
                # l0 = 1
                # l1 = 1
                # l2 = 1
                # l3 = 1
                # l4 = 1
                A1 = self.calc_dh_trans(0,0,0,l0)
                # A2 = self.calc_dh_trans(0,math.pi/2,q1+math.pi,l1)
                # A3 = self.calc_dh_trans(l2,0,q2+math.pi/2,0)
                # A4 = self.calc_dh_trans(l3,0,q3-math.pi/2,0)
                A2 = self.calc_dh_trans(0,math.pi/2,q1,l1)
                A3 = self.calc_dh_trans(l2,0,q2+math.pi/2,0)
                A4 = self.calc_dh_trans(l3,0,q3-math.pi/2,0)
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
        
        def calc_ik(self, position, orientation):
               #ASSUMING ALL LINKS HAVE LENGTH OF 1
               l1 = l2 = l3 = 1
               #get rotation matrix from quaternion data
               #rotation_matrix = tf2_py.Quaternion(orientation).to_matrix()

               #put position data into array
               position_array = np.array(position)
               xc = position_array[0]
               yc = position_array[1]
               zc = position_array[2]
                #calculate the values for each joint angle using geometrical approach
               theta1 = math.atan2(xc,yc)
               #using side and top view of manipulator, obtain values for tangent inputs
               r = math.sqrt(xc**2 + yc**2) + (math.pi)/2
               p = r - l2
               h = math.sqrt(l3**2 + p**2)
               s = zc - l1 - h
                #calculate theta 2 and 3
               theta2 = math.atan2(r,s) + (math.pi)/2
               theta3 = math.atan2(p,h)

               return theta1*(180/math.pi), theta2*(180/math.pi), theta3*(180/math.pi)


        


               
        
#initialize ros and start spinning the node   
def main(args=None):
    rclpy.init(args=args)
    node = calc_rrr_fk_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()       


                
