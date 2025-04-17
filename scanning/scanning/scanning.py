import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
import math
import time
from builtin_interfaces.msg import Time

from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped

class ScanNode(Node):
    def __init__(self):
        super().__init__('scan_node')
        print("STARTING SCAN NODE")
        
        # print("BRUH")
        # Publisher for Twist message
        self.twist_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
 
        # Timer for periodic publishing of twist messages
        self.timer_period = 0.00625  # PER SECOND
 
        self.end_effector_pose_sub = self.create_subscription(PoseStamped, '/tcp_pose_broadcaster/pose', self.tcp_callback, 10)
        self.end_effector_pose_sub  # remove "unused variable" warning

        self.wrench_sub = self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', self.wrench_callback, 10)
        self.wrench_sub

        # subscriptions should update these
        self.wrench = None
        self.tcp_pose = None

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

    
    def tcp_callback(self, pose):
        # self.get_logger().info(f"Pose callback!")
        self.tcp_pose = pose

    def wrench_callback(self, w):
        # self.get_logger().info(f"Wrench callback!")
        self.wrench = w


    def timer_callback(self): 
        if  self.wrench is None or self.tcp_pose is None:
            # Don't do anything yet
            # self.twist_publisher.publish(twist_msg)
            self.get_logger().info(f'Waiting for wrench and pose: {self.wrench} {self.tcp_pose}')
            return
        
        # Create and publish Twist message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()  # Current time
        twist_msg.header.frame_id="ur5e_tool0"

        # 1. First, try getting it to just do 0 force (?)

        # twist_msg.twist = self.fake_freedrive()
        twist_msg.twist = self.apply_downward_force_and_translate()

        # Publish the twist message
        self.twist_publisher.publish(twist_msg)

    def apply_downward_force_and_translate(self):
        twist_msg = Twist()
        target_force = -3
        err = self.wrench.wrench.force.z - target_force
        kpDown = 1.0
        if abs(err) > 0.5:
            twist_msg.linear.z = self.d(kpDown * err)

        # self.get_logger().info(f'driving with kp={kp}\terr={err}\tdrive={kp*err}')

        # Move in X direction with constant speed
        twist_msg.linear.x = self.d(3)

        # twist_msg.linear.z = self.d(-2.0) # kp * err
        return twist_msg




    # Build Twist message to do a "fake freedrive" mode, kind of
    def fake_freedrive(self):
        twist_msg = Twist()

        # Could make this a lot cleaner but whatever
        if self.wrench.wrench.force.x > 10:
            twist_msg.linear.x = self.d(-8)
        if self.wrench.wrench.force.y > 10:
            twist_msg.linear.y = self.d(-8)

        if self.wrench.wrench.force.x < -10:
            twist_msg.linear.x = self.d(8)
        if self.wrench.wrench.force.y < -10:
            twist_msg.linear.y = self.d(8)
        return twist_msg

def main(args=None):
    rclpy.init(args=args)
    node = ScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

