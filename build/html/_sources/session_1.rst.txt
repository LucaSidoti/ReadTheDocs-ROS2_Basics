Session 1
=========

Introduction
------------

In this hands-on lab, we will dive into the exciting world of ROS2, exploring its project structure and core components. You will learn how to build a ROS2 project from scratch, focusing on key elements such as nodes, topics, publishers, subscribers, services, and interfaces.

Throughout this session, we will guide you step-by-step to ensure you understand how these components work together. You will also discover essential debugging tools that will help you troubleshoot and refine your projects.

Feel free to follow our guidelines closely and complete the tasks as they come up. Remember, if you ever feel stuck or have questions about any concepts, do not hesitate to reach out for help. We are here to support you on this journey!

ROS2 Installation
-----------------

Up until now, we have mentioned that ROS2 is pre-installed in the Docker environment. Now it is time to confirm that it is actually there. To do this, we will use some demo packages that come with the ROS2 installation. Follow these steps to check it out:

1. Open **Terminator** and split the window into two terminals

2. In one of the terminals, run the following command:

.. code-block:: bash
    
    ros2 run demo_nodes_cpp talker

3. In the second terminal, run this command:

.. code-block:: bash
    
    ros2 run demo_nodes_py listener

What you should see is that the C++ node publishes a message to a topic, while the Python node listens to it.

.. warning::
    
   Do not panic if that did not make much sense right now. We will cover these concepts during today’s session. No stress!

.. important::
    
   You have successfully verified that ROS2 is installed and seen how programs in different languages (Python and C++) communicate through ROS2, showing its **language-agnostic** feature. 
   
   For the rest of the course, we will focus on **Python**.

However, that is not the only thing that has been pre-installed for your comfort. In fact, a few lines have also been added to a configuration file called *.bashrc*. This file is a script that executes every time you open a new terminal, in order to configure the terminal's local environment. Let's take a closer look:

1. Open the *.bashrc* file

.. code-block:: bash
    
    code ~/.bashrc

2. Scroll to the bottom of the script, here you should see the following lines:

.. code-block:: bash
    
    source /opt/ros/humble/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

3. Comment the *source /opt/ros/humble/setup.bash* and save the file

4. Close the previous terminals and open a **new** one

5. Run the same talker node as before

.. code-block:: bash
    
    ros2 run demo_nodes_cpp talker

**Question:** What do you observe? 

6. Now run the source command in the terminal and run the *talker* node again

.. code-block:: bash
    
    source /opt/ros/humble/setup.bash
    ros2 run demo_nodes_cpp talker

**Question:** What do you conclude? What does the source command do?

Once you have understood the meaning of this command, go back to *.bashrc* file, uncomment the *source* command and save the file.

**Question:** What is the benefit of having the *source* command in *.bashrc*?

.. tip::

   If you have been copying and pasting commands into the terminal, that is great! But remember the *tab completion tricks* we covered in the preparatory work. These also work with ROS2 commands. Give it a try by typing the following in your terminal:

   .. code-block:: bash
    
      ros2 run de

   Then, double-tap ``tab``. You should see the available options. Now type **m** and press ``tab`` again. The command will autocomplete! Use these tricks to type faster.

Previously, we also mentioned a second line that was added to your convenience. We will see what does the *source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash* command do later in the course when the time is right.


ROS2 Project Build
------------------

Now that you have verified ROS2 is installed on your desktop, let’s create a ROS2 project! The first step is to set up a workspace, which is essentially a directory where you will store your project. Within this directory, you will need to create a source folder where your application development will take place. Your workspace will also contain other folders, which we will explore later.

Let’s create the workspace and its source folder in your home directory:

.. code-block:: bash

    cd ~
    mkdir -p ros2_basics_ws/src

Next, verify that the directory was created successfully and that it contains the source folder.

As mentioned earlier, all your applications will be developed inside the source folder. Within this folder, you will add packages related to your project. Packages are the fundamental building blocks in ROS2 that help structure your project. They organize and group related functionalities and are designed to be modular and reusable, allowing you to use them in other projects.

Packages can contain a variety of elements, including nodes, libraries, configuration files, launch files, interfaces, documentation, and more. While this might seem overwhelming at first, we will clarify these concepts during the session. Let's dive in and create our first Python package:

.. code-block:: bash

    cd ~/ros2_basics_ws/source/
    ros2 pkg create ros2_basics_pkg --build-type ament_python

You can now open the source folder in VSCode and take a look at what was generated when you created the package:

.. code-block:: bash

    code .

Let’s go over the essential files you will need to pay attention to as you start working with ROS2:

1. *setup.py*: This file is where you will declare your executable ROS2 programs

2. *package.xml*: This file defines the dependencies for your package

3. *ros2_basics_pkg*: This folder will contain your package's code and resources

Again, these concepts will become much clearer when we will create our first program.

The final topic for this section is how to build a project. ROS2 provides a build tool called *colcon*, which is used to compile packages within a ROS2 workspace. It manages dependencies and ensures everything is properly linked. Let’s go ahead and build your first project:

.. code-block:: bash

    cd ~/ros2_basics_ws/
    colcon build

.. important::

    Make sure you are in the **workspace** directory before running the build command!

Once the build is complete, list the contents of your workspace. If the build worked successfully, you should see four folders: */src*, */build*, */log*, and */install*. You have already created the source folder yourself, but the other three are generated during the build process:

* */log*: Contains the logs of the compilation process
* */build*: Stores intermediate files, tests, and temporary data
* */install*: This is the most important folder, as it holds the compiled files necessary for execution

There are also additional useful commands to build your project:

1. To build a specific package from your workspace:

   .. code-block:: bash

      colcon build --packages-select <pkg_name>

2. To create symbolic links to your files:

   .. code-block:: bash

      colcon build --symlink-install

..

    This requires a bit more explanation. Normally, when you run *colcon build*, it creates copies of your package’s files in the */install* folder (specifically in the */share* directory). ROS2 only recognizes the files in the */install* folder, so any changes you make will not be applied until you rebuild the project.

    However, using the *colcon build --symlink-install* command creates symbolic links to the files in the */src* folder. This allows ROS2 to track changes in your files automatically, so you do not have to rebuild the project each time.

    .. warning::

        While this command is convenient, we recommend not using it initially so you can get familiar with the building process.

At this point, we can also explain the second line that was added to your *.bashrc* file. The *source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash* command enables *tab completion* for *colcon* commands. Try it yourself! Comment or uncomment this line in your *.bashrc* file, then try typing a *colcon* command in you terminal with the ``tab`` key to see the difference.

First Node
----------

It is finally time to start working with your first ROS2 program! In this chapter you will create your first node. A node is an independent program that handles a specific task in a ROS2 system. It is the main brick of the ROS2 environment. In a concrete project you will have multiple nodes that can communicate through topics or services (let's keep this for later) in order to achieve the desired tasks. Enough talking, let's go to work:

1. Move into the development package that we created before

.. code-block:: bash

    cd ~/ros2_basics_ws/src/ros2_basics_pkg/ros2_basics_pkg

2. Create a empty file

.. code-block:: bash

    touch minimal_node.py

3. Make it executable

.. code-block:: bash

    chmod +x minimal_node.py

4. Add the following content inside your file

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

..

  Let’s break down the key components of this minimal ROS2 node:

  a. **Imports**

  * ``rclpy``: ROS2 core Python library
  * ``Node``: Base class to define a ROS2 node

  b. **Class Definition**

  * ``MinimalNode`` inherits from ``Node``, enabling access to ROS2 functionalities
  * Written in Object-Oriented Programming (OOP) style

  c. **Constructor (__init__)**

  * Initializes the node with ``super().__init__("node_name")``, ensures that the node is correctly registered in the ROS2 system, and assigns it a name
  * Logs a message when the node starts

  d. **Main Function**

  * ``rclpy.init()``: Starts ROS2 communication
  * ``rclpy.spin(minimal_node)``: You will find out for yourself soon!
  * ``destroy_node()`` and ``rclpy.shutdown()``: Once the node is no longer required, it is properly destroyed and ROS2 communications are interrupted

5. As mentionned in the previous section, we also need to:

  a. Define dependencies in *package.xml*

  If you look at the imports in *minimal_node.py*, you can notice that the only dependency that we have is the *rclpy* library. Therefore we need to add the following line in *package.xml*:

  .. code-block:: bash

    <depend>rclpy</depend>

  b. Declare the file as a ROS2 executable program in *setup.py*

  This is done by adding an entry point as follow:

  .. code-block:: python

    entry_points={
        'console_scripts': [
            "first_node = ros2_basics_pkg.minimal_node:main"    
        ],

  .. important::

    The general structure for the entrypoints is as follow:

    .. code-block:: python

      entry_points={
          'console_scripts': [
              "<executable_name_1> = <pkg_name>.<file_name_1>:main",
              "<executable_name_2> = <pkg_name>.<file_name_2>:main"       
          ],
    
    Be sure to understand the difference between *executable_name*,  *file_name* and  *node_name*. When you will run a program from your terminal you will use the *executable_name* so be sure to know which one it is. Moreover, we would like to point out the ``,`` between the lines of the *entry_points* when your package contains multiple programs.

6. Build the project and run the node

Now you are finally ready to build and run your first node! 

  a. Open a terminal, navigate to the workspace and build your project

  b. Try to run the node with the following command:

  .. code-block:: bash

    ros2 run ros2_basics_pkg first_node

  .. note::

    The general structure to run a ROS2 node is:

    .. code-block:: bash

      ros2 run <pkg_name> <executable_name>

  .. error::

      At this point, you might encounter an unexpected message in your terminal. Can you guess what could be the problem? Ask for help and discuss with your assistant of the potential issue and how to solve it!
      
  ..

    c. After having successfully run the node, you can kill it with ``Ctrl+C`` 

  7. Comment the spin function

  Go back to *minimal_node.py* and comment ``rclpy.spin(minimal_node)``. Save the changes, build the project and run the program again.

  **Question:** What difference do you observe?

Topics Overview
---------------

Let's move forward in our journey and add communication in this project. Let's start by taking a closer look at the talker-listener example:

1. Open **Terminator** and split the window into three terminals

2. In one of the terminals, run the following command:

.. code-block:: bash
    
    ros2 run demo_nodes_cpp talker

3. In another one, run this command:

.. code-block:: bash
    
    ros2 run demo_nodes_py listener

4. In the last terminal, run simply:

.. code-block:: bash

    rqt_graph

As you can see, **rqt_graph** is a powerful tool in the ROS2 ecosystem that allows you to visualize the interactions between nodes. In our example, you can see that the *talker* node publishes messages to the *chatter* topic. Meanwhile, the *listener* node subscribes to this topic, enabling it to receive those messages.

Now that you have had a taste of what communication can look like in ROS2, let's delve a little into the theory behind it. 

In the simple **talker-listener** communication example, you saw the interaction between two key components: **nodes** and a **topic**.

- A **topic** is a communication channel used by nodes to exchange messages.
- A node acting as a **publisher** can send data to a topic.
- A node acting as a **subscriber** can receive data from a topic.

The data exchanged through a topic is called a **message**, which follows a defined **data structure**.

.. figure:: img/topics.gif
    :align: center
    :width: 90%

    `Communication via Topics <https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html>`_ 

.. admonition:: Key aspects

    * A **topic** is defined by a **name** and a **message type**.
    * Topics are used for continuous, unidirectional communication.
    * A topic can have **multiple publishers and subscribers**.
    * ROS2 communication is **anonymous**: publishers and subscribers exchange data via topics without knowing the existence of each other.

Now that you have a better understanding of how topic communication works in ROS2, let's create our own publishers and subscribers:

1. Using the same approach as before, create two new Python files: *publisher.py*, *subscriber.py*

2. Add the following code for the publisher

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

**Question:** What are the essential elements of a publisher?

3. Run the publisher node

Update the *setup.py* and *package.xml* files, then build the project and run the node. If you have any questions about these steps, please refer to the previous section or ask for assistance.

4. Examine the system

Open a new terminal and explore various ROS2 Command Line Interface (CLI) tools to inspect the system. Additionally, use ``rqt_graph``  to visualize what is happening.

+-----------------------------------------+----------------------------------------+
| CLI                                     | Command                                |
+=========================================+========================================+
| List all nodes                          | ``ros2 node list``                     |
+-----------------------------------------+----------------------------------------+
| Get details on a node                   | ``ros2 node info <node_name>``         |
+-----------------------------------------+----------------------------------------+
| List all topics                         | ``ros2 topic list``                    |
+-----------------------------------------+----------------------------------------+
| List all topics + message type          | ``ros2 topic list -t``                 |
+-----------------------------------------+----------------------------------------+
| Display messages published to a topic   | ``ros2 topic echo <topic_name>``       |
+-----------------------------------------+----------------------------------------+

After inspection, kill the node by pressing ``Ctrl+C``.

.. note::

    These tools are essential for debugging your system, allowing you to check node status, view active topics, and analyze message flow. They also help identify message types and understand node interactions, making it easier to spot issues and ensure your ROS2 applications run smoothly.

We have finished with the publisher node for now, let's move on to the subscriber node:

5. Add the following code for the subscriber

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

**Question:** What are the essential elements of a subscriber?

6. Run the publisher node

Update the *setup.py* and *package.xml* files, then build the project and run the node.

7. Examine the system

Open a new terminal and explore various ROS2 Command Line Interface (CLI) tools to inspect the system. Additionally, use ``rqt_graph``  to visualize what is happening.

+--------------------------------+-----------------------------------------------------------------+
| CLI                            | Command                                                         |
+================================+=================================================================+
| List all nodes                 | ``ros2 node list``                                              |
+--------------------------------+-----------------------------------------------------------------+
| Get details on a node          | ``ros2 node info <node_name>``                                  |
+--------------------------------+-----------------------------------------------------------------+
| List all topics                | ``ros2 topic list``                                             |
+--------------------------------+-----------------------------------------------------------------+
| List all topics + message type | ``ros2 topic list -t``                                          |
+--------------------------------+-----------------------------------------------------------------+
| Publish a message to a topic   | ``ros2 topic pub <topic_name> <msg_type> "{msg_field: 'msg'}"`` |
+--------------------------------+-----------------------------------------------------------------+

After inspection, kill the node by pressing ``Ctrl+C``.

8. Run the publisher and subscriber simultaneously

At this stage, you can run both the *publisher* and *subscriber* at the same time. Utilize the commands introduced earlier to inspect the nodes and topics. Additionally, use ``rqt_graph`` to visualize the communication.

9. Manage multiple publishers

Keep the *publisher* and *subscriber* running, then launch a **second publisher** using the usual command.

**Question:** What happens when we run two publishers with the same node name?

.. tip::

    You can use ``ros2 node list`` or ``ros2 node info`` to help you answer this question. 

In some scenarios, it may be useful to run the same executable with different node names. ROS2 allows this through a feature called **remapping**. Here is the syntax for the run command:

.. code-block:: bash

    ros2 run <pkg_name> <executable_name> --ros-args -r __node:=new_node_name

.. admonition:: Task

    Launch two publishers and two subscribers, each with unique names. Verify that you get the expected results using the CLI commands and ``rqt_graph``.

10. Remap topic at runtime

Just like nodes, topics can be renamed at runtime using remapping. You can achieve this with the following command:

.. code-block:: bash

    ros2 run <pkg_name> <executable_name> --ros-args -r default_topic_name:=new_topic_name

.. admonition:: Task

    Experiment with different publishers and subscribers by using remapping to create a graph similar to the one shown below.

    .. image:: img/task2.png
      :align: center
      :width: 80%

Exercise 1
----------

Services Overview
-----------------

Custom Interfaces
-----------------

Exercise 2
----------