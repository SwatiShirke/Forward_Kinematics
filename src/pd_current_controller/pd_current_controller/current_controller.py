import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from dynamixel_sdk_custom_interfaces.srv import GetCurrent
from dynamixel_sdk_custom_interfaces.msg import SetCurrent
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from rclpy.qos import QoSProfile
import time
import numpy as np
import math
import os
from interfaces.srv import GetRefAngle
import csv

class PD_Controller(Node):
    qos = QoSProfile(depth=10)
    def __init__(self):
        super().__init__('pd_controller')
        self.Kp = 0.003
              ## Proportional gain value 
        self.Kd = 0.002                            ## Differential gain value
        self.Ki = 0.0
        self.last_error = 0                        ## for q4 joint; used for derivative term in  PD controller
        self.error_threshold = 0.01                ## adjust this vale as imperically, it is devaition of  current angle form required angle in radians
        self.max_current = 70                    ## max value of current in Ampere the motor can withstand 
        self.prev_time = 0                         ## last value of time when controller output was aplied to the motor; used for derivative term in  PD controller
        self.ref_client = self.create_client(GetRefAngle, 'get_ref_angle')
        self.current_client = self.create_client(GetCurrent, 'get_current')
        self.get_position_client = self.create_client(GetPosition, 'get_position')
        self.last_current_passed = 0

        self.motor_id = 14
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.client_futures = []
        self.ref_angle_ticks = 0
        self.cur_angle_ticks = 0
        self.robot_motor_current = 0
        self.sum_error = 0

        #log to csv file
        self.csv_file_path = 'controller_output.csv'
        self.csv_file = open(self.csv_file_path,'w',newline='')
        self.csv_writer = csv.writer(self.csv_file)

        #init set position publisher
        self.curr_publisher = self.create_publisher(SetCurrent, 'set_current', self.qos)

    def get_ref_angle_callback(self, request, response):
        response.ref_joint_angle = math.radians(90.0)
        self.get_logger().info('Incoming request for reference angle')
        return response

    def timer_callback(self):
        get_position_req = GetPosition.Request()
        get_position_req.id = self.motor_id
        future = self.get_position_client.call_async(get_position_req)
        self.client_futures.append(future)

        ref_angle_req = GetRefAngle.Request()
        self.client_futures.append(self.ref_client.call_async(ref_angle_req))

        get_current_req = GetCurrent.Request()
        get_current_req.id = self.motor_id
        self.client_futures.append(self.current_client.call_async(get_current_req))
        
        self.pd_controller_inner()

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            incomplete_futures = []
            for f in self.client_futures:
                if f.done():
                    res = f.result()
                    if isinstance(res, GetPosition.Response):
                        self.cur_angle_ticks = res.position
                        self.get_logger().info(f"Joint Positions: \t  {self.cur_angle_ticks}")
                    elif isinstance(res, GetRefAngle.Response):
                        self.ref_angle_ticks = res.angle
                        self.get_logger().info(f"Ref Angle Ticks: \t  {self.ref_angle_ticks}")
                    elif isinstance(res, GetCurrent.Response):
                        self.robot_motor_current = res.current
                        self.get_logger().info(f"current: \t  {self.robot_motor_current}")
                else:
                    incomplete_futures.append(f)
            self.client_futures = incomplete_futures
    
    def pd_controller_inner(self):
        #current angle minus reference angle
        error = self.ref_angle_ticks - self.cur_angle_ticks
        self.sum_error += error
        #calc kd
        if self.prev_time == 0:
            Kd_term = 0
        else:
            Kd_term = self.Kd * ( error - self.last_error  ) 
        
        #calc required current
        required_current =  (self.Kp * error) + Kd_term  
        
        self.prev_time = time.time()
        #send required current to the robot
        self.publish_current_values(required_current)
        
            
        
    def publish_current_values(self,current_value): 
        # edit from shane 12/12
        msg = SetCurrent()
        msg.id = self.motor_id # only controlling the fourth motor
        print(current_value)
        # msg.current = int(max(current_value,100))
        # msg.current = int(current_value)
        msg.current = int(np.clip(current_value, -self.max_current, self.max_current))
        
        self.csv_writer.writerow([msg.current])

        self.curr_publisher.publish(msg)
        # self.get_logger().info("Published Current Values")
        
        #swati's code
        ##clipping the current value to avoid current exceeding the limit in either direction
        #current_value = np.clip(current_value, -self.max_current, self.max_current)

        #last_sign = sign(self.last_current_passed)
        #current_sign = sign(current_value)

        #if  (last_sign != current_sign):
        #    current_value = 0      ## avoid quick reverse of a current, make current zero for a while
        #   ##send current to bot
        #    rospy.sleep(0.01) 
        #else:
            ##send current to bot

        #self.last_current_passed = current_value

        ##send current value
        
        


# Initialize ros and start spinning the node   
def main(args=None):
    rclpy.init(args=args)
    node = PD_Controller()
    #node.req = "get angle"
    #angle = node.ref_client.call_async(node.req)
    #print(angle)
    # rclpy.spin(node)
    node.spin()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()       
