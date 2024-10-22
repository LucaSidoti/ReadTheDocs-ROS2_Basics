Preparatory Work 3
==================

Your ROS2 journey is almost complete, but before we dive into the final session, it is important to solidify your understanding of key concepts. In the final session, you will be working on a mini-project that will require you to apply your knowledge of topics and services. To ensure you are fully prepared, we have designed a challenge that will help you review these concepts using one of the most iconic ROS2 packages: **turtlesim**.

This challenge will serve as a foundation for the skills you will need in the mini-project, allowing you to approach the final task with confidence.


Turtlesim Overview
----------------------

The *turtlesim* package is a well-known tool for beginners in ROS2, often used to introduce core concepts. It simulates a simple turtle in a 2D space, which you can control using various ROS2 tools. 

Now it is time for you to try this turtle simulation:

1. Open a terminal and launch the *turtlesim* node with the following command:

.. code-block:: bash

    ros2 run turtlesim turtlesim_node

A window with a turtle in the center should appear.

2. In a second terminal, start the *teleop_turtle* node to control the turtle:

.. code-block:: bash

    ros2 run turtlesim turtle_teleop_key

This allows you to control the turtle using your keyboard's arrow keys: ``◄``, ``▲``, ``▼``, ``►``. Try moving the turtle around the screen.

.. warning::

    If the turtle does not move, ensure that the terminal running the *teleop_turtle* node is active. Click inside the terminal and try again.

3. Explore topics and services

Now that you have turtlesim running, let’s explore some of the topics and services it offers.

First, list the available topics:

.. code-block::

    ros2 topic list

This will display the topics that can be used to interact with the turtle. Two of them will be particularly useful for you.

Next, list the available services:

.. code-block::

    ros2 service list

The *turtlesim* package provides several services. You will use one of these to modify the turtle’s behavior.


Turtlesim Challenge
-------------------

**Goal**

* Run the turtlesim node and make the turtle move in a circular path from the center to the boundaries of the Turtlesim window with another node
* The turtle should stop near the boundaries
* Additionally, change the pen color in each quadrant of the window as the turtle moves

.. figure:: img/turtlesim_challenge.png
    :align: center
    :width: 50%

    Turtlesim challenge final result

**Steps**

1. Determine the window boundaries 
  
  Identify [xmin, xmax] and [ymin, ymax] 

  .. tip::

    Use the teleop_key_node to explore the window

2. Move the turtle 

  Send velocity commands at a defined frequency to make the turtle move away from the center

3. Boundary detection 

  Stop the turtle when it reaches the boundary limits

4. Pen color change 

  Use a *turtlesim* service to change the pen color as the turtle enters each new quadrant

  .. warning::

    Only call the service when the turtle transitions between quadrants, not continuously.


.. note::

    Great tutorial to review all the notion we have seen but with turtlesim: https://www.youtube.com/watch?v=Gg25GfA456o

