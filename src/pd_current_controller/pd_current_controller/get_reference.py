from interfaces.srv import GetRefAngle
from math import radians

import rclpy
from rclpy.node import Node

class Get_Ref_Angle_Service(Node):

    def __init__(self):
        super().__init__('get_ref_angle_service')
        self.srv = self.create_service(GetRefAngle, 'get_ref_angle', self.get_ref_angle_callback)

    def get_ref_angle_callback(self, request, response):
        response.angle = 1800.0
        self.get_logger().info('Incoming request for reference angle')
        return response


def main():
    rclpy.init()
    get_ref_angle_service = Get_Ref_Angle_Service()
    rclpy.spin(get_ref_angle_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
