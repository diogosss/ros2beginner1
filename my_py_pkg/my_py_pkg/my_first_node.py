#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__("py_test")
        self.counter_ = 0
        self.get_logger().info("Hello World Constructor")
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello: " + str(self.counter_))
        self.counter_ += 1


def main(args=None):
    rclpy.init(args=args)
    #
    #node = Node("py_test") #nombre del nodo
    #node.get_logger().info("Hello World")
    #
    node = MyNode()
    rclpy.spin(node) #keep node alive
    rclpy.shutdown

if __name__ == "__main__":
    main()