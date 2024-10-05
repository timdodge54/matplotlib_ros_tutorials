#! /usr/bin/env python3

import threading
import typing

import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
import numpy.typing as npt
import rclpy
from test_msg.msg import Test
from rclpy.subscription import Subscription
from rclpy.node import Node


class Example_Node(Node):
    """Example Node for showing how to use matplotlib within ros 2 node
    
    Attributes:
        fig: Figure object for matplotlib
        ax: Axes object for matplotlib
        x: x values for matplotlib
        y: y values for matplotlib
        lock: lock for threading
        _sub: Subscriber for node
    """

    def __init__(self):
        """Initialize."""
        super().__init__("example_node")
        # Initialize figure and axes and save to class
        self.fig, self.ax = plt.subplots()
        # create Thread lock to prevent multiaccess threading errors
        self._lock = threading.Lock()
        # create initial values to plot
        self.x = [i for i in range(5)]
        self.width = [0.5 for i in range(5)]
        # create subscriber
        self.cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self._sub: Subscription = self.create_subscription(
            Test, "test", self._callback, 10, callback_group=self.cbg
        )

    def _callback(self, msg: Test):
        """Callback for subscriber
        
        Args:
            msg: message from subscriber
                Message format
                ----------------
                int32 num
        """
        # lock thread
        with self._lock:
            # update values
            number: int = msg.num
            self.x.append(number)
            self.width.append(0.5)
            # update counter

    def plt_func(self, _):
        """Function for for adding data to axis.

        Args:
            _ : Dummy variable that is required for matplotlib animation.
        
        Returns:
            Axes object for matplotlib
        """
        # lock thread
        with self._lock:
            x = np.array(self.x)
            y = np.array(self.width)
            self.ax.barh(y, 0.5, left=x, color="red")

            return self.ax

    def _plt(self):
        """Function for initializing and showing matplotlib animation."""
        self.ani = anim.FuncAnimation(self.fig, self.plt_func, interval=1000)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = Example_Node()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    node._plt()


if __name__ == "__main__":
    main()
