Cheatsheet 
==========

ROS2 Commands
-------------

Workspace & Package Management
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    .. raw:: html

        <div class="table-centered">

    +--------------------------------------+------------------------------------------------------------------+
    | CLI                                  | Command                                                          |
    +======================================+==================================================================+
    | Set up workspace                     | ``mkdir -p ~/<ws_name>/src``                                     |
    +--------------------------------------+------------------------------------------------------------------+
    | Create Python package                | ``ros2 pkg create <pkg_name> --build-type ament_python``         |
    +--------------------------------------+------------------------------------------------------------------+
    | Create C++ package                   | ``ros2 pkg create <pkg_name> --build-type ament_cmake``          |
    +--------------------------------------+-----------------+------------------------------------------------+
    | Build all packages                   | ``colcon build``| Should be run in: **~/<ws_name>**              |
    +--------------------------------------+-----------------+------------------------------------------------+
    | Build specific package               | ``colcon build --packages-select <pkg_name>``                    |
    +--------------------------------------+------------------------------------------------------------------+
    | Build package with symlink install   | ``colcon build --packages-select <pkg_name> --symlink-install``  |
    +--------------------------------------+------------------------------------------------------------------+
    | Source workspace                     | ``source ~/<ws_name>/install/setup.bash``                        |
    +--------------------------------------+------------------------------------------------------------------+

    .. raw:: html

        </div>

Running & Launching Nodes
~~~~~~~~~~~~~~~~~~~~~~~~~

    .. raw:: html

        <div class="table-centered">

    +--------------------------------------+--------------------------------------------------------------+
    | CLI                                  | Command                                                      |
    +======================================+==============================================================+
    | Run a node                           | ``ros2 run <pkg_name> <executable_name>``                    |
    +--------------------------------------+--------------------------------------------------------------+
    | Launch nodes and configurations      | ``ros2 launch <pkg_name> <launch_file_name>``                |
    +--------------------------------------+--------------------------------------------------------------+

    .. raw:: html

        </div>


Debugging & Inspection
~~~~~~~~~~~~~~~~~~~~~~

    .. raw:: html

        <div class="table-centered">

    +-----------------------------------------+-----------------------------------------------------------------+
    | CLI                                     | Command                                                         |
    +=========================================+=================================================================+
    | List all nodes                          | ``ros2 node list``                                              |
    +-----------------------------------------+-----------------------------------------------------------------+
    | Get details on a node                   | ``ros2 node info <node_name>``                                  |
    +-----------------------------------------+-----------------------------------------------------------------+
    | List all topics + message type          | ``ros2 topic list -t``                                          |
    +-----------------------------------------+-----------------------------------------------------------------+
    | Get details on a message type           | ``ros2 interface show <msg_type>``                              |
    +-----------------------------------------+-----------------------------------------------------------------+
    | Display messages published to a topic   | ``ros2 topic echo <topic_name>``                                |
    +-----------------------------------------+-----------------------------------------------------------------+
    | List all services + service type        | ``ros2 service list -t``                                        |
    +-----------------------------------------+-----------------------------------------------------------------+
    | Get details on a service type           | ``ros2 interface show <srv_type>``                              |
    +-----------------------------------------+-----------------------------------------------------------------+

    .. raw:: html

        </div>

Tools & Utilities
~~~~~~~~~~~~~~~~~

    .. raw:: html

        <div class="table-centered">

    +-----------------------------------------+-----------------------------------------------------------------+
    | CLI                                     | Command                                                         |
    +=========================================+=================================================================+
    | Launch the RViz visualization tool      | ``rviz2``                                                       |
    +-----------------------------------------+-----------------------------------------------------------------+
    | Show the ROS2 node graph in rqt         | ``rqt_graph``                                                   |
    +-----------------------------------------+-----------------------------------------------------------------+
    | Start the Gazebo simulation environment | ``gazebo``                                                      |
    +-----------------------------------------+-----------------------------------------------------------------+

    .. raw:: html

        </div>


ROS2 Basic Structures
---------------------

Minimal Node
~~~~~~~~~~~~

    .. code-block:: python

        import rclpy
        from rclpy.node import Node

        class MinimalNode(Node):
            def __init__(self):
                super().__init__("node_name")
                self.get_logger().info("Minimal Node has been started")

        def main(args=None):
            rclpy.init(args=args)
            minimal_node = MinimalNode()
            rclpy.spin(minimal_node)
            minimal_node.destroy_node()
            rclpy.shutdown()

        if __name__ == "__main__":
            main()

Minimal Publisher
~~~~~~~~~~~~~~~~~

        .. code-block:: python

            import rclpy
            from rclpy.node import Node

            from std_msgs.msg import String

            class MinimalPublisher(Node):

                def __init__(self):
                    super().__init__('minimal_publisher')
                    self.publisher_ = self.create_publisher(String, 'topic', 10)
                    timer_period = 0.5  # seconds
                    self.timer = self.create_timer(timer_period, self.timer_callback)
                    self.i = 0

                def timer_callback(self):
                    msg = String()
                    msg.data = 'Hello World: %d' % self.i
                    self.publisher_.publish(msg)
                    self.get_logger().info('Publishing: "%s"' % msg.data)
                    self.i += 1

            def main(args=None):
                rclpy.init(args=args)
                minimal_publisher = MinimalPublisher()
                rclpy.spin(minimal_publisher)
                minimal_publisher.destroy_node()
                rclpy.shutdown()

            if __name__ == '__main__':
                main()

Minimal Subscriber
~~~~~~~~~~~~~~~~~~

    .. code-block:: python

        import rclpy
        from rclpy.node import Node

        from std_msgs.msg import String

        class MinimalSubscriber(Node):
            def __init__(self):
                super().__init__('minimal_subscriber')
                self.subscription = self.create_subscription(String,'topic',
                                                            self.listener_callback, 10)
                self.subscription  # prevent unused variable warning

            def listener_callback(self, msg):
                self.get_logger().info('I heard: "%s"' % msg.data)

        def main(args=None):
            rclpy.init(args=args)
            minimal_subscriber = MinimalSubscriber()
            rclpy.spin(minimal_subscriber)
            minimal_subscriber.destroy_node()
            rclpy.shutdown()

        if __name__ == '__main__':
            main()

Minimal Server
~~~~~~~~~~~~~~

    .. code-block:: python

        import rclpy
        from rclpy.node import Node

        from example_interfaces.srv import AddTwoInts

        class MinimalServer(Node):
            def __init__(self):
                super().__init__('minimal_server')
                self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

            def add_two_ints_callback(self, request, response):
                response.sum = request.a + request.b
                self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

                return response

        def main():
            rclpy.init()
            minimal_server = MinimalServer()
            rclpy.spin(minimal_server)
            minimal_server.destroy_node()
            rclpy.shutdown()

        if __name__ == '__main__':
            main()

Minimal Client
~~~~~~~~~~~~~~

    .. code-block:: python

        import rclpy
        from rclpy.node import Node

        from functools import partial
        from example_interfaces.srv import AddTwoInts


        class MinimalClient(Node):
            def __init__(self):
                super().__init__("minimal_client")
                self.call_add_two_ints_server(6, 7)

            def call_add_two_ints_server(self, a, b):
                client = self.create_client(AddTwoInts, "add_two_ints")
                while not client.wait_for_service(1.0):
                    self.get_logger().warn("Waiting for Server Add Two Ints...")

                request = AddTwoInts.Request()
                request.a = a
                request.b = b

                future = client.call_async(request)
                future.add_done_callback(partial(self.callback_call_add_two_ints, a=a, b=b))

            def callback_call_add_two_ints(self, future, a, b):
                try:
                    response = future.result()
                    self.get_logger().info(str(a) + " + " + str(b) + " = " + str(response.sum))
                except Exception as e:
                    self.get_logger().error("Service call failed %r" % (e,))

        def main(args=None):
            rclpy.init(args=args)
            minimal_client = MinimalClient()
            rclpy.spin(minimal_client)
            minimal_client.destroy_node()
            rclpy.shutdown()

        if __name__ == "__main__":
            main()
