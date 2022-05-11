from cmath import pi
import sys

sys.path.append('/home/k3lso/Desktop/k3lso_A/src/k3lso_light')
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from controllers.mpc.mpc_controller import MPCController
from model.robots.k3lso.k3lso_mpc import K3lso
from imu import IMU
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
import numpy as np
import time

        # Publisher for pose controller
class MinimalPublisher(Node):

    def __init__(self):
        #self.imusub = MinimalSubscriber()
        self.imu = IMU(np.array([0.0, 0.0, 0.0]))
        self.robot = K3lso(self.imu, "1", None)
        # self.controller = MPCController(self.robot, 0)    # Initialize a controller object


        super().__init__('mpc_publisher')
        self.publisher = self.create_publisher(
            Float32MultiArray, 
            '/mpc', 
            10)
        
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.imu_sub_callback,
            10)
        

        # Callback for publisher
    def timer_callback(self):
        # Gör en array som läser av guit
        # self.controller.update_signal(self,"GUI ARRAY")
        msg = Float32MultiArray()       # Create msg to send 
        msg.data = self.imu.orientation.tolist()

        self.publisher.publish(msg)

    def imu_sub_callback(self, msg):
        self.imu.update([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
            [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

# Subscriber for IMU msg for feedback
# class MinimalSubscriber(Node):

#     def __init__(self):
#         super().__init__('imu_subscriber')
#         self.subscription = self.create_subscription(
#             Imu,
#             'imu',
#             self.imu_sub_callback,
#             10)
#         self.subscription  # prevent unused variable warning
#         self.data = []

#     def imu_sub_callback(self, msg):
#         self.data = msg
#         print(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    #minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_publisher)
    #rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #minimal_subscriber.destroy_node()
    minimal_publisher.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()