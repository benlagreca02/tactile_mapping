import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
import math
import time
from builtin_interfaces.msg import Time

from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped

class FakeFreedrive(Node):
    def __init__(self):
        super().__init__('FakeFreedrive')
        self.get_logger().info("FakeFreedrive node created")
        
        # print("BRUH")
        # Publisher for Twist message
        self.twist_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
 
        # Timer for periodic publishing of twist messages
        self.timer_period = 0.0025  # PER SECOND
 
        self.wrench_sub = self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', self.wrench_callback, 10)
        self.wrench_sub

        self.wrench = None

        self.get_logger().info("Requesting servoing...")
        self.start_servoing()
        self.get_logger().info("Servoing activated, Activate interlock! Dyna-therms connected! Infra-cells up; mega-thrusters are go!")
        self.timer = self.create_timer(self.timer_period, self.timer_callback)  

    def d(self, speed):
        return speed * self.timer_period

    def start_servoing(self):
        #  Tell the servo node to start servoing
        self.cli = self.create_client(Trigger, '/servo_node/start_servo')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, checking again in a second...')
        req = Trigger.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.servo_service_callback)


    def servo_service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Service call succeeded: {response.message}")
            else:
                self.get_logger().warn(f"Service call failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


    def wrench_callback(self, w):
        # self.get_logger().info(f"Wrench callback!")
        self.wrench = w


    def timer_callback(self): 
        if  self.wrench is None:
            # Don't do anything yet
            # self.twist_publisher.publish(twist_msg)
            # self.get_logger().info(f'Waiting for wrench and pose: {self.wrench} {self.tcp_pose}')
            return
        
        # Create and publish Twist message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()  # Current time

        # 1. First, try getting it to just do 0 force (?)

        # twist_msg.twist = self.fake_freedrive()
        twist_msg.twist = self.fake_freedrive()

        # Publish the twist message
        self.twist_publisher.publish(twist_msg)

    # Build Twist message to do a "fake freedrive" mode, kind of
    def fake_freedrive(self):
        twist_msg = Twist()

        THRESH = 15
        BASE = -0.9
        # Could make this a lot cleaner but whatever
        x,y,z = self.wrench.wrench.force.x, self.wrench.wrench.force.y, self.wrench.wrench.force.z
        if x > THRESH:
            twist_msg.linear.x = self.d(BASE*x)
        elif x < -THRESH:
            twist_msg.linear.x = self.d(BASE*x)

        if y > THRESH:
            twist_msg.linear.y = self.d(BASE*y)
        elif y < -THRESH:
            twist_msg.linear.y = self.d(BASE*y)

        if z > THRESH:
            twist_msg.linear.z = self.d(-BASE*z)
        elif z < -THRESH:
            twist_msg.linear.z = self.d(-BASE*z)

        return twist_msg

def main(args=None):
    rclpy.init(args=args)
    node = FakeFreedrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

