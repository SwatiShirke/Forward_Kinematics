import numpy as np
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node

# from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import SetJointPosition
from interfaces.srv import CalibratePosition, ApplyDeltaQ, GiveEefVelocity, GiveJointVelocity

from transforms3d import quaternions

class IncrementalController(Node):
    qos = QoSProfile(depth=10)
    
    def __init__(self):
        super().__init__('incremental_controller')
        self.get_logger().info("Starting the Incremental Controller Node")

        self.measured_joint_values = [0,0,0,0]
        self.current_joint_values = [0,0,0,0]
        self.goal_joint_values = [0, 0, 0, 0]

        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.apply_delta_callback)

        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, self.qos)
        self.create_service(CalibratePosition, 'calibrate_position', self.calibrate_position)
        self.create_service(ApplyDeltaQ, 'apply_delta_q', self.apply_delta_q)
        self.goal_joint_space = self.create_client(SetJointPosition, 'goal_joint_space_path')

    def joint_state_callback(self, msg):
        self.measured_joint_values = msg.position[:-1]
    
    def calibrate_position(self, request, response):
        self.current_joint_values = self.measured_joint_values 
        response.success = True
        return response

    def apply_delta_q(self, request, response):
        joint_deltas = request.joint_position_deltas
        self.get_logger().info(f'Joint Deltas = {joint_deltas}')
        self.current_joint_values = np.array(self.current_joint_values) + np.array(joint_deltas)
        
        self.goal_joint_space_req = SetJointPosition.Request()
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        self.goal_joint_space_req.joint_position.position = self.current_joint_values.tolist()
        self.get_logger().info(f'Setting Joint Positions = \n{self.current_joint_values.tolist()}')
        self.goal_joint_space_req.path_time = 1.0 # sample time

        try:
            self.goal_joint_space.call_async(self.goal_joint_space_req)
            response.success = True
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))
            response.success = False
            
        return response
   

# Initialize ros and start spinning the node   
def main(args=None):
    rclpy.init(args=args)
    node = IncrementalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()       


                
