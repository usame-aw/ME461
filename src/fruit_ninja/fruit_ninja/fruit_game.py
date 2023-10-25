#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from example_interfaces.msg import Float64MultiArray

def main(args=None):

    def finger_pos_cb(msg):
        node.get_logger().info(f"{msg.data[0]} {msg.data[1]}")

    rclpy.init(args=args)
    node = Node("fruit_game")
    sub = node.create_subscription(Float64MultiArray, "/fruit_game/finger_pos", finger_pos_cb, 10)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
