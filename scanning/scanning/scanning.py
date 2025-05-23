import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
import math
import time
from builtin_interfaces.msg import Time

from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped

from controller_manager_msgs.srv import SwitchController
import tf_transformations as tf

class ScanNode(Node):
    def __init__(self):
        super().__init__('scan_node')
        self.get_logger().info("Scan node created.")
        
        # print("BRUH")
        # Publisher for Twist message
        self.twist_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
 
        # Timer for periodic publishing of twist messages
        self.timer_period = 0.00125  # PER SECOND
 
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

        # Main timer loop
        self.t0 = time.time()
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
        rclpy.spin_until_future_complete(self, future)
    
    def tcp_callback(self, pose):
        # self.get_logger().info(f"Pose callback!")
        self.tcp_pose = pose

    def wrench_callback(self, w):
        # self.get_logger().info(f"Wrench callback!")
        self.wrench = w

    def is_aligned(self, tolerance) -> bool:
        # TODO FIX THIS
        return False
        if self.tcp_pose is None:
            return False

        # Get Quaternion of tool
        bruh = self.tcp_pose.pose
        qu = self.tcp_pose.pose.orientation
        curr = [qu.x, qu.y, qu.z, qu.w]

        # Our "known good" one 
        target = [0.0, 1.0, 0.0, 0.0]

        delta = []
        for q, t in zip(curr, target):
            delta.append(abs(q-t))

        self.get_logger().info(f'{delta}')
        if max(delta) > tolerance:
            return False

        return True


    def timer_callback(self): 
        if  self.wrench is None or self.tcp_pose is None:
            # Don't do anything yet
            # self.twist_publisher.publish(twist_msg)
            # self.get_logger().info(f'Waiting for wrench and pose: {self.wrench} {self.tcp_pose}')
            self.ready_to_start = True
            self.scanning = False
            return

        # Create and publish Twist message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()  # Current time
        # This is spec. in the armconfig servoing yaml
        # twist_msg.header.frame_id="tip"

        # twist_msg.twist = self.test_drive()
        # twist_msg.twist = self.apply_downward_force_and_translate()  
        if not self.is_aligned(0.5):
            # self.get_logger().info("aligning...")
            twist_msg.twist = self.align_z()
        else:
            # self.get_logger().info("Done!")
            pass


        # TODO make funciton for "align along Z axis so tip is upright"
        # TODO make bound checking, so "apply_downward..." func stops
        # TODO make "reset tip" funciton to move us back to the start
        # TODO keep GIANT list of all points
        # TODO feed that list to some STL generate code
        # TODO go home and quit

        # Publish the twist message
        self.twist_publisher.publish(twist_msg)
        # self.print_tcp_pose()
        

    def print_tcp_pose(self):
        point = self.tcp_pose.pose.position
        x,y,z = point.x, point.y, point.z
        quat = self.tcp_pose.pose.orientation
        rx = quat.x
        ry = quat.y
        rz = quat.z
        rw = quat.w
        self.get_logger().info(f"TIP POSE: {x:.4f} {y:.4f} {z:.4f}\tQuat: {rx:.4f} {ry:.4f} {rz:.4f} {rw:.4f}")


    def normRPY(self, theta):
        return (theta + math.pi) % (2*math.pi) - math.pi

    def align_z(self):
        quat = self.tcp_pose.pose.orientation
        rx = quat.x
        ry = quat.y
        rz = quat.z
        rw = quat.w
        twist_msg = Twist()

        kpr = 400
        kpp = 400
        kpy = 400
        
        current = [rx, ry, rz, rw]
        # target = [0.0, 1.0, 0.0, 0.0]

        cr, cp, cy = tf.euler_from_quaternion(current)
        # tr, tp, ty = tf.euler_from_quaternion(target)

        
        tr = math.pi if cr > 0 else -math.pi
        tp = 0  
        ty = math.pi if cy > 0 else -math.pi

        dr = tr-cr
        dp = tp-cp
        dy = ty-cy


        twist_msg.angular.x = self.d(dr * -kpr)
        twist_msg.angular.y = self.d(dp * kpp) # idk frames or something
        twist_msg.angular.z = self.d(dy * -kpy)
        
        # self.get_logger().info(f'Tr: {tr:.3f} \t Cr: {cr:.3f} \t Drpy: {dr:.3f}')
        self.get_logger().info(f'Trpy: {tr:.3f} {tp:.3f} {ty:.3f} \t Crpy: {cr:.3f} {cp:.3f} {cy:.3f} \t Drpy: {dr:.3f} {dp:.3f} {dy:.3f}')

        return twist_msg


    def apply_downward_force_and_translate(self):
        fz = self.wrench.wrench.force.z 
        fx = self.wrench.wrench.force.x 
        tx = self.wrench.wrench.torque.x 
        ty = self.wrench.wrench.torque.y 
        twist_msg = Twist()

        # Looks like when hanging still, force is 29
        # Pretty noisy, but hangs round 29.0 - 29.4
        # TRIM = 29.3
        target_force_z = -10.0  # Force "target" in z direction
        kpz = 0.5             # Factor for 'correction' Main driver in Up/down

        FORCE_Z_TOL = 0.1
        FORCE_X_TOL = 1.0
        kpx = 0.8   # speed in L-R based on force in X
        kpxz = 0.3  # slowdown in U-D based on force in X
        kpr = 10.0   # rotation speed based on force in X
        BASE_X_SPEED = 4.0  # CHANGE THIS TO CHANGE DIRECTION

        # forceErrZ = fz - target_force_z - TRIM
        forceErrZ = fz - target_force_z 


        twist_msg.linear.z = self.d(forceErrZ * kpz)

        # set a base speed, this will be increased, or decreased based on forces
        twist_msg.linear.x = self.d(BASE_X_SPEED)

        # slow down in x if force in x is high IN EITHER DIRECTION
        twist_msg.linear.x -= self.d(fx * kpx * (1 if fx > 0 else -1)) 
        # twist_msg.linear.x -= self.d(fx * kpx * (1 if fx > 0 else 1)) 

        # move up faster if fx is bigger, down faster if fx < 0
        # twist_msg.linear.z += self.d(fx * -kpxz)

        # rotate accordingly
        if abs(fx) > FORCE_X_TOL:
            twist_msg.angular.y = self.d(fx*kpr)

        # self.get_logger().info(f'fz: {fz:.4f}\t Adjusted: {(fz-TRIM):.3f}\tforceErrZ: {forceErrZ:.4f}')
        # self.get_logger().info(f'fx: {fx:.4f}')
        # self.get_logger().info(f'forceErrZ: {forceErrZ:.4f}\tfx: {fx:.4f}')
        return twist_msg


    def test_drive(self):
        '''
        Wiggles end effector to ensure its moving wrt the tip
        '''
        twist_msg = Twist()
        # bruh = 450*math.sin(2 * math.pi * 0.125 * time.time())
        # twist_msg.angular.y = self.d(bruh)
        # twist_msg.linear.x = self.d(50)
        # twist_msg.linear.z = self.d(10)
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

