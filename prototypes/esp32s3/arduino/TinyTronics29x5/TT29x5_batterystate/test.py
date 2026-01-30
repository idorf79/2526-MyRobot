#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32, Float32

class GreetingsPublisher(Node):
    def __init__(self):
        super().__init__("battery_state_to_29x5")
        self.publisher_ = self.create_publisher(Float32, "test", 1)
#        self.timer_ = self.create_timer(2.0, self.publish_greetings)
        self.subscription = self.create_subscription(
            BatteryState,
            'battery_state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.voltage)
        voltageMsg = Float32()
        voltageMsg.data = msg.voltage
        self.publisher_.publish(voltageMsg)


    def publish_greetings(self):
        msg = Float32()
        msg.data = 3.02
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GreetingsPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
