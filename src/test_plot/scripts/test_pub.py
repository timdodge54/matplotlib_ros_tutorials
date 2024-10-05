#! /usr/bin/env python3
import time

import rclpy
import rclpy.callback_groups
from test_msg.msg import Test
from rclpy.client import Client
from rclpy.node import Node
import numpy as np


class TestPub(Node):
    def __init__(self) -> None:
        super().__init__("test_pub")
        self.cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.pub = self.create_publisher(Test, "test", 10, callback_group=self.cbg)
        self.counter = 4
        time.sleep(5)
        self.timer = self.create_timer(1, self.loop_pub)

    def loop_pub(self) -> None:
        self.counter += 1
        msg = Test()
        msg.num = np.random.randint(0, 30)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestPub()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
