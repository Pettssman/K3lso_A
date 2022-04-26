import sys
sys.path.append('/home/k3lso/Desktop/k3lso_A/src/k3lso_light')
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pose import Pose
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
import time

        # Publisher for pose controller
class MinimalPublisher(Node):

    def __init__(self):
        self.controller = Pose()        # Initialize a controller object
        super().__init__('pose_publisher')
        self.publisher = self.create_publisher(
            Float32MultiArray, 
            '/pose', 
            10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Callback for publisher
    def timer_callback(self):
        # Gör en array som läser av guit
        # self.controller.update_signal(self,"GUI ARRAY")
        msg = Float32MultiArray()       # Create msg to send 
        data = self.controller.check_gui()
        self.controller.update_signal(data[0],data[1])      # Update signal from GUI
        msg.data = self.controller.get_signal()     # Update msg.data with motor positions
        # msg.data = [1.00002,2.93938]      # Test
        self.publisher.publish(msg)


        # Subscriber for IMU msg for feedback
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.imu_sub_callback,
            10)
        self.subscription  # prevent unused variable warning


    def imu_sub_callback(self, msg):
        msg

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_publisher)
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
