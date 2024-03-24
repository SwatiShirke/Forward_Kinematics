import sys, time, math
from pprint import pformat

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter 
from rclpy.qos import QoSProfile

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose

import numpy as np
from transforms3d import quaternions

from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.msg import KinematicsPose
from interfaces.srv import GiveInverseKinematics, CalibratePosition, ApplyDeltaQ, GiveJointVelocity
from sensor_msgs.msg import JointState

class Utils(Node):
    qos = QoSProfile(depth=10)

    def __init__(self):
        super().__init__('utils',
                            allow_undeclared_parameters=True,
                            automatically_declare_parameters_from_overrides=True)

        self.get_logger().info("Running Utils")

        command_param = self.get_parameter_or('cmd', Parameter('str', Parameter.Type.STRING, 'help'))
        joint_positions = self.get_parameter_or('joint_positions', Parameter('joints', Parameter.Type.DOUBLE_ARRAY, [0.0,0.0,0.0,0.0]))
        orient = self.get_parameter_or('orientation', Parameter('joints', Parameter.Type.DOUBLE_ARRAY, [1.0,0.0,0.0,0.0]))
        position = self.get_parameter_or('position', Parameter('joints', Parameter.Type.DOUBLE_ARRAY, [0.0,0.0,0.0]))

        xy1 = self.get_parameter_or('from', Parameter('from', Parameter.Type.DOUBLE_ARRAY, [0.0, 0.0]))
        y_delta = self.get_parameter_or('delta', Parameter('delta', Parameter.Type.DOUBLE , 0.0))

        self.client_futures = []
        cmd = str(command_param.value)
        if cmd == "help":
            self.help()
        elif cmd == "home":
            self.home()
            sys.exit()
        elif cmd == "test":
            self.test()
        elif cmd == "print_joint_states":
            self.print_joint_state()
        elif cmd == "print_inv_kine":
            self.print_inv_kinematics()
        elif cmd == "move":
            self.move(joint_positions.value)
            sys.exit()
        elif cmd == "calc_pose":
            self.print_calculated_pose()
            sys.exit(0)
        elif cmd == "inv_kine":
            self.inverse_kinematics(orient.value, position.value)
            sys.exit(0)
        elif cmd == "pick_and_place":
            self.pick_and_place(xy1.value)
            sys.exit(0)
        elif cmd == "ymove":
            self.ymove(y_delta.value)
        else:
            self.get_logger().info("Params: %s" % (cmd))
        
    
    def help(self):
        print('''\tUtils
              1) home: Set Robot to Home State
              ''')
        sys.exit()

    def home(self):
        print('''\tUtils
              Setting Robot to home state
              ''')
        self.goal_joint_space = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.goal_joint_space_req = SetJointPosition.Request()

        self.tool_control = self.create_client(SetJointPosition, 'goal_tool_control')
        self.tool_control_req = SetJointPosition.Request()


        ##############################
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        self.goal_joint_space_req.joint_position.position = [0.0, 0.0, 0.0, 0.0]
        self.goal_joint_space_req.path_time = 5.0

        try:
            self.goal_joint_space.call_async(self.goal_joint_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))
        ##############################

        self.tool_control_req.joint_position.joint_name = ['gripper']
        self.tool_control_req.joint_position.position = [0.9]
        self.tool_control_req.path_time = 5.0

        try:
            self.tool_control_result = self.tool_control.call_async(self.tool_control_req)
        except Exception as e:
            self.get_logger().info('Tool control failed %r' % (e,))
        

    def move(self, joint_values):
        print(f'''\tMove
              Setting Robot to Joint state: {joint_values}
              ''')

        self.goal_joint_space = self.create_client(SetJointPosition, 'goal_joint_space_path')

        goal_joint_space_req = SetJointPosition.Request()
        goal_joint_space_req.planning_group = '0'
        goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        goal_joint_space_req.joint_position.position = joint_values
        goal_joint_space_req.path_time = 5.0
        try:
            future = self.goal_joint_space.call_async(goal_joint_space_req)
            rclpy.spin_until_future_complete(self, future)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))
    
    def move_to_pose(self, orientation, position):
        self.get_logger().info("inverse kine start")
        joint_values = self.inverse_kinematics(orientation, position)
        self.get_logger().info(f"Joint values {joint_values}")
        self.move(joint_values)
    
    def actuate_gripper(self, state):
        # state \in [0,1]
        self.goal_tool_control = self.create_client(SetJointPosition, 'goal_tool_control')
        new_state = state*0.18-0.09

        goal_tool_control_req = SetJointPosition.Request()
        goal_tool_control_req.joint_position.joint_name = ['gripper']
        goal_tool_control_req.joint_position.position = [new_state]
        goal_tool_control_req.path_time = 1.0
        try:
            future = self.goal_tool_control.call_async(goal_tool_control_req)
            rclpy.spin_until_future_complete(self, future)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

        
    def print_calculated_pose(self):
        self.create_subscription(Pose, 'calculated_pose', self.forward_kinematics_callback, self.qos)
    
    def inverse_kinematics(self, orientation, position):
        inv_kine = self.create_client(GiveInverseKinematics, 'give_inverse_kinematics')
        msg = GiveInverseKinematics.Request()
        msg.pose.orientation.w = orientation[0]
        msg.pose.orientation.x = orientation[1]
        msg.pose.orientation.y = orientation[2]
        msg.pose.orientation.z = orientation[3]
        msg.pose.position.x =  position[0]
        msg.pose.position.y =  position[1]
        msg.pose.position.z =  position[2]
        self.get_logger().info(f"msg inv_kine: {msg}")

        try:
            future = inv_kine.call_async(msg)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
        #Get response
            self.get_logger().info(f'The current Joint values are: {response.joint_positions}')
            return response.joint_positions
        except Exception as e:
            self.get_logger().info('Sending Inv Kine msg failed %r' % (e,))
            return [0.0, 0.0, 0.0, 0.0]


    def forward_kinematics_callback(self, msg):
        self.pose = msg
        print(f"\n\n\n\n\n\n\n\nPose: \n\tOrientation: {self.pose.orientation}\n\tPosition: {self.pose.position}")
       
    def pick_and_place(self, xy1):
        self.get_logger().info(f'The From position is: {xy1}')   
        orientation = np.array([
            [0, 0, -1],
            [-1, 0, 0],
            [0, 1, 0],
        ])

        position = [124.0, 0.0 , 91.0]

        # orientation_quat = quaternions.mat2quat(orientation)
        orientation_quat = [0.5, 0.5, -0.5, -0.5]   

        self.home()
        self.move_to_pose(orientation_quat, position)

    def test(self):
        self.create_subscription(KinematicsPose, 'kinematics_pose', self.inv_kine_callback, self.qos)
    
    def print_inv_kinematics(self):
        self.inv_kine = self.create_client(GiveInverseKinematics, 'give_inverse_kinematics')
        self.create_subscription(KinematicsPose, 'kinematics_pose', self.inv_kine_callback, self.qos)
    
    def inv_kine_callback(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation

        msg = GiveInverseKinematics.Request()
        msg.pose.orientation.w = round(orientation.w,3)
        msg.pose.orientation.x = round(orientation.x, 3)
        msg.pose.orientation.y = round(orientation.y, 3)
        msg.pose.orientation.z = round(orientation.z, 3)
        # robot gives in meters
        msg.pose.position.x =  round(position.x*1000, 3)
        msg.pose.position.y =  round(position.y*1000, 3)
        msg.pose.position.z =  round(position.z*1000, 3)

        try:
            self.client_futures.append(self.inv_kine.call_async(msg))
        except Exception as e:
            self.get_logger().info('Sending Inv Kine msg failed %r' % (e,))

    def print_joint_state(self):
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, self.qos)
    
    def joint_state_callback(self, msg):
        joint_name = msg.name
        joint_positions = msg.position
        joint_states = zip(joint_name, joint_positions)
        output = ', '.join(f'{round(f,2)}' for f in joint_positions[:-1])
        self.get_logger().info(f'\nJoint States: \t\t\t {output}')
    
    def ymove(self, y_target):
        self.get_logger().info(f"Moving y by {y_target} dist")
        timer_period = 1 # seconds
        self.ymove_progress = 0;
        self.ymove_increment = math.copysign(10, y_target)*4;
        self.ymove_target = y_target ;

        self.give_joint_velocities = self.create_client(GiveJointVelocity, 'give_joint_velocities')
        self.calibrate_position = self.create_client(CalibratePosition, 'calibrate_position')
        self.apply_delta_q = self.create_client(ApplyDeltaQ, 'apply_delta_q')

        self.calibrate_position.call_async(CalibratePosition.Request())
        self.timer = self.create_timer(timer_period, self.ymove_callback)

    
    def ymove_callback(self):
        actual_inc = min(abs(self.ymove_target - self.ymove_progress), abs(self.ymove_increment))
        actual_inc = math.copysign(actual_inc, self.ymove_increment)
        self.get_logger().info(f"yMove_Callback with position ref: {actual_inc}")
        self.ymove_progress += actual_inc
        if (abs(self.ymove_progress - self.ymove_target) < 0.1):
            self.timer.cancel()

        give_joint_velo_req = GiveJointVelocity.Request()
        give_joint_velo_req.eef_velocities = [0.0, actual_inc*1.0, 0.0, 0.0 ,0.0 ,0.0]

        self.get_logger().info(f"Sent EEF Velocities {give_joint_velo_req.eef_velocities}")

        self.client_futures.append(self.give_joint_velocities.call_async(give_joint_velo_req))

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            incomplete_futures = []
            for f in self.client_futures:
                if f.done():
                    res = f.result()
                    if isinstance(res, GiveInverseKinematics.Response):
                        joint_positions = res.joint_positions
                        self.get_logger().info(f"Joint Positions: \t  {[ round(x, 2) for x in joint_positions ]}")
                    elif isinstance(res, GiveJointVelocity.Response):
                        joint_velocities = res.joint_velocities
                        apply_delta_q = ApplyDeltaQ.Request()
                        self.get_logger().info(f" actual position {[round(p,2) for p in  joint_velocities]}")
                        apply_delta_q.joint_position_deltas = joint_velocities.tolist()
                        self.get_logger().info(f"Sent Joint Velocities {[round(r,2) for r in joint_velocities]}")
                        self.apply_delta_q.call_async(apply_delta_q)
                else:
                    incomplete_futures.append(f)
            self.client_futures = incomplete_futures
    

# Initialize ros and start spinning the node   
def main(args=None):
    rclpy.init(args=args)
    node = Utils()
    # rclpy.spin(node)
    node.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()       


                
