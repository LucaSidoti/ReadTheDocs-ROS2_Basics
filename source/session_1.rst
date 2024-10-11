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

However, that is not the only thing that has been pre-installed for your convenience. In fact, a few lines have also been added to a configuration file called *.bashrc*. This file is a script that executes every time you open a new terminal, in order to configure the terminal's local environment. Let's take a closer look:

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

First Node
----------

Topics Overview
---------------

Exercise 1
----------

Services Overview
-----------------

Custom Interfaces
-----------------

Exercise 2
----------