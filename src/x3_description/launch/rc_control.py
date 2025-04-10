#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCIn
from geometry_msgs.msg import Twist


class RCToCmdVel(Node):
    def __init__(self):
        super().__init__('rc_to_cmdvel')

        self.declare_parameter('throttle_channel', 2)
        self.declare_parameter('yaw_channel', 3)
        self.declare_parameter('pitch_channel', 1)
        self.declare_parameter('roll_channel', 0)

        self.rc_sub = self.create_subscription(RCIn, '/mavros/rc/in', self.rc_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def rc_callback(self, msg):
        try:
            twist = Twist()

            # Channel mappings (adjust if your transmitter is different)
            throttle = msg.channels[self.get_parameter('throttle_channel').value]
            yaw = msg.channels[self.get_parameter('yaw_channel').value]
            pitch = msg.channels[self.get_parameter('pitch_channel').value]
            roll = msg.channels[self.get_parameter('roll_channel').value]

            # Map RC [1000-2000] to [-1, 1] or suitable range
            def scale(val): return (val - 1500) / 500.0

            twist.linear.z = scale(throttle) 
            twist.angular.z = -scale(yaw)
            twist.linear.x = scale(pitch)*2
            twist.linear.y = -scale(roll)*2

            self.cmd_pub.publish(twist)

        except IndexError:
            self.get_logger().warn("RC channels out of range")


def main(args=None):
    rclpy.init(args=args)
    node = RCToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()