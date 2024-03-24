import math
import numpy as np

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node

from geometry_msgs.msg import Pose,Point,Quaternion
from open_manipulator_msgs.msg import KinematicsPose 
from sensor_msgs.msg import JointState
from interfaces.srv import GiveInverseKinematics  

from transforms3d import quaternions

from geometry_msgs.msg import Pose,Point,Quaternion
from sensor_msgs.msg import JointState

class InverseKinematics(Node):
    qos= QoSProfile(depth=10)

    def __init__(self):
        super().__init__('inverse_kinematics')
        self.get_logger().info("Starting Inverse Kinematics Node")

        self.srv = self.create_service(GiveInverseKinematics, 'give_inverse_kinematics', self.give_inverse_kinematics)   
    
    def give_inverse_kinematics(self, request, response):
        #extract end effector position and orientation from input pose

        self.get_logger().info("Got Request for Inverse Kinematics")
        self.desired_pose = request
        x = self.desired_pose.pose.orientation.x
        y = self.desired_pose.pose.orientation.y
        z = self.desired_pose.pose.orientation.z
        w = self.desired_pose.pose.orientation.w
        # desired_pose_rot = self.quaternion_rotation_matrix(w,x,y,z)
        desired_pose_rot = quaternions.quat2mat([w,x,y,z])
        desired_position = [self.desired_pose.pose.position.x, self.desired_pose.pose.position.y, self.desired_pose.pose.position.z]

        #pass values to inverse kinematics function
        theta1, theta2, theta3, theta4 = self.calculate_inverse_kinematics(desired_pose_rot,desired_position)
        response.joint_positions = [theta1, theta2, theta3, theta4]
        self.get_logger().info("Sent response")
        self.desired_pose = request
        return response

    def calculate_inverse_kinematics(self,orientation,position):
        """ Function to calculate joint angles theta1-theta4 for a desired EE pose

        Input: 
        :param orientation:4x4 end effector rotation matrix
        :param postion: 3x1 array with end effector x,y,z position

        Output:
        Joint angles for J1-J4 in degrees
        """
        #end effector positions
        xe = position[0]
        ye = position[1]
        ze = position[2]
        #link lengths
        l0 = 36.076
        l1 = 96.326-36.076
        l2 = math.sqrt(128**2 + 24**2)
        l3 = 124
        l4 = 133.4

        x_vector = np.array([orientation[0][0], orientation[1][0], orientation[2][0]])

        #math.radians(gamma) = theta2+theta3+theta4 aka the orientation of the ee frame wrt to the base frame
        #not exactly sure how to extract this from the rotation matrix
        #I THINK THIS WILL WORK TO GIVE THE ANGLE BETWEEN THE X AXIS OF THE EE AND BASE FRAMES, SINCE ANY CHANGE IN
        # ORIENTATION WILL BE A ROTATION AROUND THE Y AXIS. AGREE?
        # gamma = math.acos(orientation[0,0])

        #position of J4
        o4 = np.array(position) - (l4*x_vector)
        xy4 = math.sqrt(o4[0] ** 2 + o4[1] ** 2)
        z4 = o4[2] - l0- l1
        C = math.sqrt(xy4**2 + z4**2)
        # self.get_logger().info(f'xe{xe}, ye{ye}, xy4 {xy4},l2+l3 {l2+l3} C{C}')

        z = [0, 0, 1]
        gamma =math.pi/2 - math.acos(np.dot(x_vector, z))

        # self.get_logger().info(f'position {position} o4 {o4}, h {horizontal}, gamma {gamma}, mag o {mag(o4_norm)}, mag h {mag(horizontal)}')

        #condition for cosine theorem
        if (l2+l3) > C:
             #calculate alpha and beta
             a = l2**2 + l3**2 - C**2
             alpha = math.acos((a)/(2*l2*l3))
             beta = math.acos((l2**2 + C**2 - l3**2)/(2*l2*C))

             #joint angles elbow-down configuration
             theta2a = math.atan2(z4,xy4) - beta
             theta3a = math.radians(180) - alpha
             theta4a = gamma - theta2a - theta3a

             #joint angles elbow-up configuration
             theta2b = math.atan2(z4,xy4) + beta
             theta3b = -(math.radians(180) - alpha)
             theta4b = gamma - theta2b - theta3b
        else:
             print("Error! Invalid manipulator configuration")

        # theta1 = math.degrees(math.atan2(ye,xe))
        theta1 = (math.atan2(ye,xe))

        # Elbow down
        # theta2 = math.degrees(theta2a)
        # theta3 = math.degrees(theta3a)
        # theta4 = math.degrees(theta4a)
        # theta2 = (theta2a)
        # theta3 = (theta3a)
        # theta4 = (theta4a)

        # Elbow up
        theta2 = (theta2b)
        theta3 = (theta3b)
        theta4 = (theta4b)

        # theta2 = math.degrees(theta2b- math.pi/2)
        # theta3 = math.degrees(theta3b + math.pi/2)
        # theta4 = math.degrees(theta4b)
        theta2 = (theta2b- math.pi/2)
        theta3 = (theta3b + math.pi/2)
        theta4 = (theta4b)

        return theta1, theta2,theta3,theta4
        

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



def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
