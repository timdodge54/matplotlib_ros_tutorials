# Using MatplotLib with threading in Python to allow for Seamless Real Time Data Visualization

## Explanation of Threading and Matplotlib
Matplotlib requires synchronous plotting functionality to occur on the main thread. Within ROS anything that is executed in a timer or callback is executed on a secondary thread. This means if a user is attempting to plot regularly from information that is retrieved from either service-client, or publisher-subscriber requires manual pushing of plotting to the main thread to allow for the images to be displayed.

To explain how to do this two examples are shown. The first example shows how the threading can be achieved without ROS to explain what is happening on the Python side. The second example follows similar logic to the first example but is implemented in ROS2 to show what changes are specific to ROS.

# Example of Threading Without ROS

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import threading
import time


class Tester:
    """Example of how to use matplotlib with threading.

    Attributes:
        fig: Figure object for matplotlib
        ax: Axes object for matplotlib
        x: x values for matplotlib
        y: y values for matplotlib
        counter: counter for class
        lock: lock for matplotlib
    """

    def __init__(self):
        # Initialize figure and axes and save to class
        self.fig, self.ax = plt.subplots()
        # create initial values to plot
        self.plt_x = [i for i in range(5)]
        self.plt_width = [0.5 for i in range(5)]
        self.plt_counter = 0
        self._lock = threading.Lock()

    def plt_func(self, _):
        """Function for matplotlib animation.

        Args:
            _: Dummy variable for matplotlib animation 

        Returns:
            Axes object for matplotlib
        """
        # lock thread
        with self._lock:
            x = np.array(self.plt_x)
            y = np.array(self.plt_width)
            self.ax.barh(y, 0.5, left=x, color="red")
            return self.ax

    def loop_logic(self):
        """Looping logic for adding data to plot."""
        time.sleep(3)
        print("init")
        while True:
            self.plt_add()
            time.sleep(1)

    def plt_add(self):
        """"Add data to class arrays."""
        with self._lock:
            if self.plt_counter > 4:
                self.plt_x.append(self.plt_counter)
                self.plt_width.append(0.5)
            self.plt_counter += 1
            print(self.plt_counter)

    def _plt(self):
        print("INIT PLOT")
        self.ani = anim.FuncAnimation(self.fig, self.plt_func, interval=1000)
        print("showing")
        plt.show()


if __name__ == "__main__":
    test = Tester()
    thread = threading.Thread(target=test.loop_logic)
    thread.start()
    test._plt()
```

# Explanation of Example 1

In this example, the class Tester is used to simulate a node class when used in ROS. First, in the init function, a Matplotlib figure and axis are saved to the class. Both will be continually updated throughout the lifecycle of the class. The x, and y member variables are initialized with data to show how to update existing data. A threading lock must also be created to prevent the simultaneous access of the data that is to be plotted.  

The loop logic function begins by adding data to the x and y member variables every three seconds. This loop logic calls a second function ```plt_add``` that uses the threading lock ```_lock``` to add data to the member variables x and y.

To allow for continuous updating of the axis the ```FuncAnimation``` function is used which updates the figures with a given callable at the frequency given by the keyword argument interval. Any callable used by FuncAnimation must return the axis that will update the given figure.

The function ```plt_func``` follows this convention using the threading lock to retrieve the data from the member variables x and y and adding them to the axis and returning it.

In the main function, the class is initialized and the ```loop_logic``` function is placed into a second thread and the thread is started. The ```_plt``` function is started in the main thread.

# Example Using ROS

Using this same logic the next example implements this plotting in a ROS NODE.

```python
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

```

# Major Differences Between Example 1 and 2

The main change is the loop_logic function from the previous example becomes a callback for the class's subscriber.

Then in the main function, a multi-threaded executor is created and the example node is added to the executor. The spinning of this executor is then placed into the thread and an extra keyword is added ```daemon=True``` to allow for the ROS2 daemon to execute. Otherwise, the layout is the same.

# Location of Examples and How to Run

The Example Code can be found within the ```src``` folder of this repository. To run the examples, first, clone the repository and then build the package using colcon. To build this repository you must be using ROS2 Foxy or newer. The following commands will install the required dependencies, build the package, and run the example.

```bash
cd "repository"
source /opt/ros/$ROSDISTRO/setup.bash
pip install -r requirements.txt
colcon build
. install/setup.bash
ros2 launch test_plot test.launch.py
```

# Boiler Plate Code

# Boiler Plate Code

Boiler plate code for plotting can be found on my profile and can be cloned with the following command.

```bash
git clone https://github.com/timdodge54/boiler_plate_plotting.git
```
