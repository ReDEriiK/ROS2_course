import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen


class koch_snow(Node):

    def __init__(self):
        super().__init__('koch_snow')
        self.twist_pub = self.create_publisher(
                            Twist, '/turtle1/cmd_vel', 10)
        self.pose = None
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.cb_pose,
            10)

    # New method for koch_snow
    def cb_pose(self, msg):
        self.pose = msg

    def set_pen(self, r, g, b, width, off):
        pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        while not pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_pen szolgáltatás nem elérhető, várakozás...')
            self.get_logger().info('set_pen szolgáltatás elérhető.')

        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        future = pen_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Toll beállítva.')
        else:
            self.get_logger().error('Hiba történt a toll beállításakor.')

    def go_straight(self, speed, distance):
        # Implement straght motion here
        # Create and publish msg
        vel_msg = Twist()
        if distance > 0:
            vel_msg.linear.x = speed
        else:
            vel_msg.linear.x = -speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        # Set loop rate
        loop_rate = self.create_rate(100, self.get_clock()) # Hz

        # Calculate time
        T = abs(distance / speed)

        # Publish first msg and note time when to stop
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Turtle started.')
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        # Publish msg while the calculated time is up
        while (self.get_clock().now() <= when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            #self.get_logger().info('On its way...')
            rclpy.spin_once(self)   # loop rate

        # turtle arrived, set velocity to 0
        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Arrived to destination.')

    def turn(self, omega, angle):
        # Implement straght motion here
        # Create and publish msg
        vel_msg = Twist()

        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        if angle > 0:
            vel_msg.angular.z = math.radians(omega)
        else:
            vel_msg.angular.z = -math.radians(omega)


        # Set loop rate
        loop_rate = self.create_rate(100, self.get_clock()) # Hz

        # Calculate time
        T = abs(angle / omega)

        # Publish first msg and note time when to stop
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Turtle started.')
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        # Publish msg while the calculated time is up
        while (self.get_clock().now() <= when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            #self.get_logger().info('On its way...')
            rclpy.spin_once(self)   # loop rate

        # turtle arrived, set velocity to 0
        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Arrived to destination.')

    def set_spawnpoint(self, speed, omega, x, y):
        self.set_pen(0, 0, 0, 0, 1)
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.pose is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)

        # Stuff with atan2
        x0 = self.pose.x
        y0 = self.pose.y
        theta_0 = math.degrees(self.pose.theta)

        theta_1 = math.degrees(math.atan2(y-y0, x-x0))
        angle = theta_1 - theta_0
        distance = math.sqrt(((x - x0) * (x - x0)) + (y - y0) * (y - y0))

        # Execute movement
        self.turn(omega, angle)
        self.go_straight(speed, distance)

        self.set_pen(255, 255, 255, 1, 0)
    def draw_koch(self, speed, omega, I, L):
        if I==0:
            self.go_straight(speed, L)
        else:
            self.draw_koch(speed, omega, I-1, L)
            self.turn(omega, 60)
            self.draw_koch(speed, omega, I-1, L)
            self.turn(omega, -120)
            self.draw_koch(speed, omega, I-1, L)
            self.turn(omega, 60)
            self.draw_koch(speed, omega, I-1, L)

def main(args=None):
    rclpy.init(args=args)
    ks = koch_snow()
    ks.set_spawnpoint(5.0,500.0,1,2)
    ks.turn(500, -155)
    for k in range(3):
        ks.draw_koch(1.0,600.0,4,0.5)
        ks.turn(500, -120)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ks.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
