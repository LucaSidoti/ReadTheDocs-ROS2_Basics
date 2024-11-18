Session 2
=========

Introduction
------------

Welcome to session 2 of our ROS2 Basics labs! Today, we will be exploring the visualization and simulation tools integrated with ROS2. In this session, we will dive into Rviz, Transforms (TFs), URDF, Xacro, parameters, launch files, and Gazebo, building the essential skills for designing and simulating robotic systems.

Our main goal for today is to create a model of the Thymio robot that is fully integrated with ROS2 and capable of smooth operation in Gazebo simulation. We will guide you step-by-step through the essential elements needed to achieve this. Starting with simple examples to introduce each concept, you will progressively build the Thymio model in parallel, gaining practical experience with each component as you go. Now, let's get to work!


URDF Tutorial Example
---------------------

In this first part of the session, we will start by exploring **Rviz**, a 3D visualization tool, and **TFs** (transform frames), which are essential for managing coordinate frames in ROS2. We will use an example **URDF** (Unified Robot Description Format) model to learn how to visualize and interact with a robot model in Rviz. Do not worry if these concepts do not fully make sense yet, we will go through each of them step by step throughout today’s session. 

Let’s get started by ensuring that everything we need is installed and set up. The first step is to install the *urdf_tutorial* package. This package is not part of the default ROS2 installation, so let's install it now.

1. Open a terminal and run:

.. code-block:: bash

    sudo apt install ros-humble-urdf-tutorial && source /opt/ros/humble/setup.bash

.. note::

    If you have been paying attention to the terminal messages, you may have noticed that this package is already installed in the Docker environment. Due to time considerations, we have not covered all the steps of the installation process. However, the following outlines the general procedure you can follow to install any new package in ROS2.

    +-------+----------------------------+--------------------------------------------------+
    | Step  | Task                       | Command                                          |
    +=======+============================+==================================================+
    | 1     | **Update** your package    | ``sudo apt update``                              |
    |       | list                       |                                                  |
    +-------+----------------------------+--------------------------------------------------+
    | 2     | **Upgrade** existing       | ``sudo apt upgrade``                             |
    |       | packages                   |                                                  |
    +-------+----------------------------+--------------------------------------------------+
    | 3     | **Install** the desired    | ``sudo apt install ros-<distro>-<package_name>`` |
    |       | package                    |                                                  |
    +-------+----------------------------+--------------------------------------------------+
    | 4     | **Source** the ROS2        | ``source /opt/ros/<distro>/setup.bash``          |
    |       | setup file                 |                                                  |
    +-------+----------------------------+--------------------------------------------------+
    | 5     | **Verify** the package     | ``ros2 pkg list | grep <package_name>``          |
    |       | installation               |                                                  |
    +-------+----------------------------+--------------------------------------------------+

2. Let’s check where this package and others are installed:

.. code-block:: bash

    cd /opt/ros/humble/share && ls

3. Navigate to the URDF folder of the *urdf_tutorial* package and check its content:

.. code-block:: bash

    cd /opt/ros/humble/share/urdf_tutorial/urdf && ls

Among the files listed, you will see `08-macroed.urdf.xacro`, which is the example we will use now.

4. Launch the example:

.. code-block:: bash

    ros2 launch urdf_tutorial display.launch.py model:=/opt/ros/humble/share/urdf_tutorial/urdf/08-macroed.urdf.xacro

Now that Rviz is running and the robot model is loaded, let’s take a moment to explore what Rviz offers and why it is such a valuable tool in the ROS2 ecosystem. Visualizing the robot in a 3D environment provides an intuitive way to understand its structure, relationships, and interactions.

In particular, Rviz allows you to visualize:

    * **Robot models**: These are detailed representations of the robot’s physical structure.
    * **Transforms frames (TFs)**: Coordinate frames that define the spatial relationships between different parts of the robot, essential for motion and interaction.
    * **Sensors data**:  Information from sensors such as LiDAR, cameras, or depth sensors, displayed in real-time to help interpret the robot’s environment.

As you can see in the image below, Rviz offers a variety of default plugins that enable you to monitor different aspects of your system. These plugins can be accessed and added to your workspace by clicking the ``Add`` button in the Rviz interface.

.. image:: img/rviz_plugins.png
    :align: center
    :width: 20%

.. |spacer| raw:: html

    <div style="margin-top: 5px;"></div>

|spacer|

Rviz also allows you to interact with tools like the *Joint State Publisher*, a GUI that lets you manipulate the robot’s joints. This enables you to see how joint movements affect the robot’s structure.

A key question to consider here is: **How does ROS2 determine the positions and movements of the different links relative to one another over time?**

The answer lies in **Transform Frames (TFs)**. These frames represent the spatial relationships (positions and orientations) between the robot’s parts and its environment. TFs enable ROS2 to continuously track how each part of the robot moves in relation to others. By maintaining structured relationships between frames, TFs play a crucial role in various robotic tasks.

Each frame has three axes: x (red), y (green), and z (blue), representing its orientation in 3D space. If you uncheck the ``RobotModel`` in Rviz, you can see that the TFs form a tree-like structure, showing how the robot’s rigid parts are connected.

To better visualize the TF hierarchy, you can use the *view_frames* tool provided by the *tf2_tools* package. Open a terminal and run:

.. code-block:: bash

    cd ros2_basics_ws/
    ros2 run tf2_tools view_frames 

After about five seconds, a PDF will be generated in your workspace. This file provides a clear tree structure of the robot’s TFs. The ``base_link`` is the root of the TF tree, and all other frames are connected as branches. Each branch connects a **parent link** to a **child link**, meaning that if the parent link moves, the child link will move accordingly.

Now that you understand the purpose of TFs, you can experiment in Rviz to see how they work alongside the ``RobotModel``. Start by focusing on the ``RobotModel``:

1. Hide TFs and explore the ``RobotModel`` by displaying all links or only a few.
2. Adjust joints using the *Joint State Publisher* and observe changes.

Next, enable the TFs and hide the ``RobotModel`` to focus on the transform frames:

1. Display all or specific frames to examine their relationships.
2. Observe how frames update dynamically with joint adjustments.

By now, you should have a foundational understanding of Rviz and TFs. Here’s a quick recap:

    * **Rviz** is a visualization tool that helps display robot models, TFs, and many other essential elements .
    * **TFs** are essential for representing spatial relationships and movement between different parts of the robot.

These tools are invaluable for building and visualizing robot models in ROS2.

Let’s keep going! In the next chapters, we will dive deeper into understanding and working with URDF files to create our own robot model.


URDF Overview
-------------

In this introductory example, the need for TFs (Transform Frames) in robotics was highlighted. TFs play a crucial role in tracking the positions of different parts of a robot over time. They are essential for most control packages in ROS2 to function effectively.

For example:

* **Odometry** in navigation requires the positions of the wheels to estimate a mobile robot's pose.
* **Robotic arms** need joint positions to calculate the pose of the end-effector.

In short, accurate TFs are vital for running a robot in ROS2. Fortunately, ROS2 handles the management of TFs. The only requirement is to provide a **URDF file**, which describes the robot's elements in **XML format**.

A URDF, Unified Robot Description Format, consists of two main components:

* **Links**: Represent the physical, rigid parts of the robot. These correspond to the ``RobotModel`` in Rviz.
* **Joints**: Define the relationships between links and are used by ROS2 to generate TFs.

Links are the rigid bodies of a robot. They can be described using one of four geometry types:

1. **Boxes**
2. **Cylinders**
3. **Spheres**
4. **Meshes**

.. note::

    In this class, only basic geometry shapes will be used. While meshes can be included in a URDF, they require a CAD-designed mesh file (e.g. an STL file). When using meshes, it is important to pay attention to scaling and orientation.

To fully define a link, three properties must be specified:

* **Visual**: How the link appears in visualization tools.
* **Inertial**: The physical properties (mass, center of gravity, etc.).
* **Collision**: The geometry used for collision detection.

These properties will be introduced progressively throughout the session.

.. figure:: img/links.png
   :align: center
   :width: 50%

   `Link representation <http://wiki.ros.org/urdf/XML/link>`_

Joints define the connections between links. The most common types of joints in ROS2 are:

1. **Fixed**: No movement between the parent and child links.
2. **Revolute**: Rotation around a single axis within a defined range.
3. **Continuous**: Rotation around a single axis without limits.
4. **Prismatic**: Linear motion along a single axis.

A joint is always defined by its **parent link** and **child link**.

.. figure:: img/joints.png
   :align: center
   :width: 50%

   `Joint representation <http://wiki.ros.org/urdf/XML/joint>`_

.. important::

    For more information, consult the official documentation: `Links <http://wiki.ros.org/urdf/XML/link>`_ or `Joints <http://wiki.ros.org/urdf/XML/joint>`_.


Minimal URDF - Visual
---------------------

With the necessary theoretical background in place, it is time to create our first robot model. We will start by setting up a new package to develop the **Thymio** model. Following standard conventions, the package will be named ``<robot_name>_description`` and structured as a C++ package. 

1. Create a new package:

.. code-block:: bash

    cd ~/ros2_basics_ws/src/
    ros2 pkg create thymio_description

2. Remove unnecessary folders:

.. code-block:: bash

    cd ~/ros2_basics_ws/src/thymio_description/
    rm -rf include/ src/
    ls

3. Create directories for today's materials:

.. code-block:: bash

    mkdir -p urdf/example urdf/thymio launch rviz worlds
    ls

Now, let's build the package and verify its structure.

4. Build the package:

.. code-block:: bash

    cd ~/ros2_basics_ws
    colcon build --packages-select thymio_description
    source install/setup.bash

By default, ROS2 stores all packages in the *install* directory. Inside, you will find your package along with a *share* directory.

5. Verify package installation:

.. code-block:: bash

    cd ~/ros2_basics_ws/install/thymio_description/share/thymio_description
    ls

.. error:: 

    You will notice that the folders we created are missing. This means ROS2 cannot access them. To fix this, we need to update the *CMakeLists.txt* file to include these directories in the installation process.

6. Update *CMakeLists.txt*:

Add the following block above ``if(BUILD_TESTING)``:

.. code-block:: bash

    install(
        DIRECTORY urdf launch rviz worlds
        DESTINATION share/${PROJECT_NAME}/
        )

This command ensures the directories are installed at the correct location. Rebuild the package and verify again:

.. code-block:: bash

    cd ~/ros2_basics_ws
    colcon build --packages-select thymio_description
    source install/setup.bash
    cd ~/ros2_basics_ws/install/thymio_description/share/thymio_description
    ls

Now, we are ready to create our first URDF file. We will start simple, defining a single box and displaying it in Rviz.

7. Create an *example.urdf* file:

.. code-block:: bash

    cd ~/ros2_basics_ws/src/thymio_description/urdf/example
    touch example.urdf

8. Fill the *example.urdf* file with the first link:

    a. Define the structure of the URDF file

    .. code-block:: xml

        <?xml version="1.0"?>
        <robot name="example">

        </robot>

    This structure specifies the XML format and gives a name to the robot model.

    b. Add a link with visual properties

    .. code-block:: xml 

        <?xml version="1.0"?>
        <robot name="example">
        
            <link name="base_link">
                <visual>    
                    <origin xyz="0 0 0"  rpy="0 0 0"/>
                    <geometry>
                        <box size="0.2 0.3 0.6"/>
                    </geometry>
                </visual>
            </link>

        </robot>

    The **base_link** is a standard name for the core element of a robot. Its dimensions are specified in meters. The **origin** uses **xyz** for position and **rpy** (roll, pitch, yaw) for orientation.

    .. note::

        Tags in XML must be opened (e.g. ``<visual>``) and closed (e.g. ``</visual>``). If a tag is empty, it can be written as a self-closing tag (e.g. ``<box size="0.2 0.3 0.6"/>`` instead of ``<box size="0.2 0.3 0.6"> </box>``).

    c. Visualize the box in Rviz

    First, build the package. Since more components will be added shortly, it is convenient to use the ``--symlink-install`` option for quicker updates.

    .. code-block::

        colcon build --packages-select thymio_description --symlink-install
        source install/setup.bash

    .. warning::

        This command is useful when working with URDFs as it allows you to progressively verify your progress. However, remember to rebuild the package whenever you add a new file.

    Now, we are ready to visualize the result. Use the *urdf_tutorial* package to launch the URDF in Rviz. Later, we will explore how to achieve this without relying on this package.

    .. code-block::

        ros2 launch urdf_tutorial display.launch.py model:=/home/ubuntu/ros2_basics_ws/install/thymio_description/share/thymio_description/urdf/example/example.urdf

    We have successfully created a box with dimensions: 20 cm in length (x-direction), 30 cm in width (y-direction), and 10 cm in height (z-direction). However, it appears with a default color in Rviz. Let’s modify it to add a custom color.
    
    .. warning::

        You can ignore the warning message in Rviz as it will be resolved shortly.

    d. Add color to the visual

    .. code-block:: xml

        <?xml version="1.0"?>
        <robot name="example">

            <material name="blue">
                <color rgba="0 0 1 1" />
            </material>

            <link name="base_link">
                <visual>    
                    <origin xyz="0 0 0"  rpy="0 0 0"/>
                    <geometry>
                        <box size="0.2 0.3 0.6"/>
                    </geometry>
                    <material name="blue"/>
                </visual>
            </link>

        </robot>

    Colors are defined using the `material` tag. A common practice is to define colors at the top of the file and reference them by name in the `visual` tag. The color attributes include four arguments: **rgb** for red, green, and blue, and **a** for transparency.

    View the result in Rviz using the same command as before (rebuilding the package is not necessary). 

9. Fill the *example.urdf* file with the first joint:

    To introduce joints, we will add a second link and then connect it to the base link.

    a. Define a second link

    .. code-block:: xml 

        <?xml version="1.0"?>
        <robot name="example">

            <material name="blue">
                <color rgba="0 0 1 1" />
            </material>

            <link name="base_link">
                <visual>    
                    <origin xyz="0 0 0"  rpy="0 0 0"/>
                    <geometry>
                        <box size="0.2 0.3 0.6"/>
                    </geometry>
                    <material name="blue"/>
                </visual>
            </link>

            <link name="second_link">
                <visual>
                    <origin xyz="0 0 0" rpy="0 0 0"/>
                    <geometry>
                        <cylinder length="0.8" radius="0.05"/>
                    </geometry>
                    <material name="blue"/>
                </visual>
            </link>

        </robot>

    Here, we have simply added a second link with a different shape. The name can be chosen arbitrarily. You can now try visualizing it in Rviz.

    .. error:: 

        The result cannot be visualized yet because unconnected links are not allowed. Let’s resolve this by adding a joint.


    b. Create a joint between the links

    .. code-block:: xml

        <?xml version="1.0"?>
        <robot name="example">

            <material name="blue">
                <color rgba="0 0 1 1" />
            </material>

            <link name="base_link">
                <visual>    
                    <origin xyz="0 0 0"  rpy="0 0 0"/>
                    <geometry>
                        <box size="0.2 0.3 0.6"/>
                    </geometry>
                    <material name="blue"/>
                </visual>
            </link>

            <link name="second_link">
                <visual>
                    <origin xyz="0 0 0" rpy="0 0 0"/>
                    <geometry>
                        <cylinder length="0.8" radius="0.05"/>
                    </geometry>
                    <material name="blue"/>
                </visual>
            </link>

            <joint name="second_link_joint" type="fixed">
                <parent link="base_link"/>
                <child link="second_link"/>
                <origin xyz="0 0 0" rpy="0 0 0"/>
            </joint>

        </robot>

    The second link has been added. Use a naming convention for the joint that makes it easy to identify. As mentioned earlier, a joint is defined by its **type**, **parent link**, and **child link**. Additionally, it includes an **origin**, which specifies its position and orientation relative to the parent link. Now, let’s visualize this in Rviz.

    c. Set the origins

    We have not discussed origins yet, as they can be a bit confusing when working with URDFs for the first time. To simplify, we will provide a straightforward approach to correctly position your links. This step is critical for creating a robot model that works accurately in simulation.

    As mentioned earlier, ROS2 uses the URDF file to generate the robot's TFs. If the joints are not properly placed, the TFs will also be misaligned, leading to unexpected behavior during simulation.

    Let’s go through a simple four-step process to correctly position two links. Currently, our setup looks like this:


    .. image:: img/urdf_example.png
        :align: center
        :width: 20%

    .. |spacer| raw:: html

        <div style="margin-top: 5px;"></div>

    |spacer|

    Our goal now is to replicate this: 

    .. image:: img/urdf_example_final.png
        :align: center
        :width: 50%

    .. |spacer| raw:: html

        <div style="margin-top: 5px;"></div>

    |spacer|

    For each of the following steps, update the code and visualize the result in Rviz. Reflect on the purpose of each origin setting. If you have any questions, don’t hesitate to ask, it is crucial to understand this process.
    
        1. Set all origins to zero (this is already the case)

        .. code-block:: xml

            <origin xyz="0 0 0" rpy="0 0 0"/>

        2. Set the origin for the visual of the base_link

        .. code-block:: xml

            <origin xyz="0 0 0.3"  rpy="0 0 0"/>

        3. Set the joint origin

        .. code-block:: xml

            <origin xyz="0.1 0 0.3" rpy="0 0 0"/>

        4. Set the origin for the visual of the second_link

        a. Rotation

        .. code-block:: xml

            <origin xyz="0 0 0" rpy="0 1.57 0"/>

        b. Translation

        .. code-block:: xml

            <origin xyz="0.4 0 0" rpy="0 1.57 0"/>

    **Question**: Which origin setting is most critical for ensuring that your robot's movements and positions are accurately represented in ROS2 simulations?

    d. Explore the different joint types

    Now, let’s experiment with the two links by trying out different joint types. Simply replace the existing joint with one of the examples below. For each joint type, visualize the result in Rviz and use the Joint State Publisher GUI to observe how the parts move. 

        1. Revolute

        .. code-block:: xml

            <joint name="second_link_joint" type="revolute">
                <parent link="base_link"/>
                <child link="second_link"/>
                <origin xyz="0.1 0 0.3" rpy="0 0 0"/>
                <axis xyz="1 0 0"/>
                <limit lower="-1.57" upper="1.57" velocity="10" effort="10"/>
            </joint>

        2. Continuous

        .. code-block:: xml

            <joint name="second_link_joint" type="continuous">
                <parent link="base_link"/>
                <child link="second_link"/>
                <origin xyz="0.1 0 0.3" rpy="0 0 0"/>
                <axis xyz="1 0 0"/>
            </joint>

        3. Prismatic

        .. code-block:: xml

            <joint name="second_link_joint" type="prismatic">
                <parent link="base_link"/>
                <child link="second_link"/>
                <origin xyz="0.1 0 0.3" rpy="0 0 0"/>
                <axis xyz="1 0 0"/>
                <limit lower="0.0" upper="0.5" velocity="10" effort="10"/>
            </joint>

Reaching this point means you now have a better understanding of what a URDF is. You are equipped with the essential tools to finally practice building your first robot model on your own. Let’s get started!

Thymio - Step 1
~~~~~~~~~~~~~~~

As mentioned in the introduction, today's goal is to create a Thymio model that works well in simulation. In addition to building the model, you will set up custom methods to launch applications like Rviz and Gazebo, ensuring they function correctly with your robot. The task is divided into 10 steps, and the journey begins now with your first challenge: creating the visual representation of the Thymio using the tools just introduced.

Start by creating a new file named *thymio.urdf* in the */urdf/thymio* directory. Use the provided specifications to guide you through the process, and remember to consistently use Rviz to visualize your progress.

.. +----------------------+----------------------------------+------------------+
.. | Component            | Specifications                   | Additional Info  |
.. +======================+==================================+==================+
.. | **base link**        | Box:                             | Ground clearance:|
.. |                      | Length = 11 cm |                 | 4.5 mm           |
.. |                      | Width = 11.2 cm |                |                  |
.. |                      | Height = 4.4 cm                  |                  |
.. +----------------------+----------------------------------+------------------+
.. | **caster wheel**     | Sphere:                          |                  |
.. |                      | Radius = 9 mm                    |                  |
.. +----------------------+----------------------------------+------------------+
.. | **wheels**           | Cylinder:                        |                  |
.. |                      | Length = 1.5 cm |                |                  |
.. |                      | Radius = 2.2 cm                  |                  |
.. +----------------------+----------------------------------+------------------+

+----------------------+---------------------------------------------+--------+
| Component            | Specifications                              | Color  |
+======================+=============================================+========+
| **Base link**        | *Box:*                                      | White  |
|                      | Length = 11 cm |                            |        |
|                      | Width = 11.2 cm |                           |        |
|                      | Height = 4.4 cm                             |        |
|                      | |spacer|                                    |        |
|                      | *Ground clearance:* 4.5 mm                  |        |
+----------------------+---------------------------------------------+--------+
| **Caster wheel**     | *Sphere:*                                   | White  |
|                      | Radius = 9 mm                               |        |
+----------------------+---------------------------------------------+--------+
| **Wheels**           | *Cylinder:*                                 | Black  |
|                      | Length = 1.5 cm |                           |        |
|                      | Radius = 2.2 cm                             |        |
+----------------------+---------------------------------------------+--------+

Refer to the following drawing to correctly place the different links:

.. image:: img/thymio_spec.png
    :align: center
    :width: 60%

.. |spacer| raw:: html

    <div style="margin-top: 5px;"></div>

|spacer|

.. admonition:: Hints

  .. toggle::

    * The ground clearance information should be sufficient to define all the heights  
    * Carefully consider where the TFs should be positioned (this is crucial!)  
    * Use the following command to visualize the model in Rviz:

    .. code-block::

        ros2 launch urdf_tutorial display.launch.py model:=/home/ubuntu/ros2_basics_ws/install/thymio_description/share/thymio_description/urdf/thymio/thymio.urdf

    * The final visual result should look like this:  

    .. image:: img/thymio_look.png
        :align: center
        :width: 60%


Improved URDF - Xacro
---------------------

Congratulations on completing the simplified Thymio model! Now, to prepare for the next step, consider this question: **What happens if we change the dimensions of the base_link?**

Try answering this by modifying the width of the ``base_link`` to 6.6 cm instead of 11.2 cm.

You will notice that the wheels are no longer correctly aligned with the sides of the ``base_link``. This is because the current URDF uses hardcoded values. Any change to one dimension requires manual updates to other dependent dimensions. While manageable for a small file, this approach is likely to cause mistakes and become inefficient for larger models.

In programming, this problem is typically solved by using variables to define relationships between dimensions, ensuring automatic updates when one value changes. While URDF does not support variables, **Xacro**, an extension of URDF, solves this issue. Xacro allows for:

* **Parametrization**: Define variables for dynamic adjustments.
* **Simplification**: Use macros, constants, math operations, and conditional logic.
* **Modularity**: Organize your robot description into multiple files.

To use Xacro, you write your file using its extended syntax and process it with the *xacro* tool to generate a complete URDF that ROS2 can use. Let’s apply this to our example and see how it works in practice.


.. If you have successfully created this simplified Thymio model, congratulations, you are ready to move on and improve this URDF. In order to motivate what we will do next, we would like you to reflect on the folowwing question "What happens if we decide to change the base_link dimensions?". For example, you can try to answer this question by changing the width of the base_link and put a width of 6.6cm instead of 11.2cm and open the model in Rviz. 

.. What you should witness is that the wheels are no longer correctly placed relative to the sides of the base_link. Indeed, for now we have been working only with hardcoded values, which means that if we want to change a dimension in our URDF we might need to rectify others dimensions as well. In our case, this might not be too difficult, but imagine what would happen if we were working if a bigger file.

.. How can we solve this? In any other programming language, what you would do is to create a variable for the base_link width and define the positions of the wheels relative to this variable, so that when you change the width the wheels are still correctly attached to the base_link. Unfortunately, this is not possible with a normal URDF. However, we can the xacro extension that is an extension of URDF that offers more advanced functionality. In short, Xacro, which stands for XML Macros, is a macro language used to simplify and manage URDF files more efficiently. Compare to URDF, Xacro brings:

.. * Parametrization: variables
.. * Simplification: macros (functions), constant (pi), math operations, conditional logics
.. * Modularity: multiple files to define one URDFs

.. To use a Xacro file in ROS2, it must first be processed by the Xacro tool. This tool takes all the Xacro files, combines and processes them, and generates a single, complete URDF file that the ROS2 system can use.

.. Enough theory, let's go back to our simple example and see how this works in practice.

1) Rename the file

First, rename the previous example file to include the Xacro extension: *example.urdf.xacro*

2) Xacro compatibility

To enable the use of xacro in our file, we need to adjust the robot tag as follow:

.. code-block:: xml

    <robot name="example" xmlns:xacro="http://wwww.ros.org/wiki/xacro">

3) Mathematical operations

Xacro enables various mathematical operations, including the use of the constant pi, often needed for adjusting link orientations. For example, the ``second_link`` origin can be rewritten as:

.. code-block:: xml

    <origin xyz="0.4 0 0" rpy="0 ${pi / 2.0} 0"/>

4) Variables

Variables can be defined like this:

.. code-block:: xml

    <xacro:property name="base_link_length" value="0.2"/>
    <xacro:property name="base_link_width" value="0.3"/>
    <xacro:property name="base_link_height" value="0.6"/>

And used as shown here:

.. code-block:: xml

    <box size="${base_link_length} ${base_link_width} ${base_link_height}"/>

5) Macros

Xacro supports defining reusable functions called macros. For example, a macro to define a box with length, width, and height as parameters can be written as:

.. code-block:: xml

    <xacro:macro name="box" params="length width height">
        <box size="${length} ${width} ${height}"/>
    </xacro:macro>

You can then call it at the desired location with the required parameters:

.. code-block:: xml

    <xacro:box length="${base_link_length}" width="${base_link_width}" height="${base_link_height}"/>

6) Multiple files

To simplify the process, it is a good practice to split the URDF into multiple files. Typically, one main file includes all other Xacro files. To distinguish them, use the extension *.urdf.xacro* for the main file and *.xacro* for the others. 

For example, let’s create a new file to define colors. Name it *example_materials.xacro* and save it in the */urdf/example* directory. This file follows the same structure as the main file but does not include a robot name. Add the following content to the new file:

.. code-block:: xml

    <?xml version="1.0"?>
    <robot xmlns:xacro="http://wwww.ros.org/wiki/xacro">

        <material name="blue">
            <color rgba="0 0 1 1"/>
        </material>

        <material name="green">
            <color rgba="0 1 0 1"/>
        </material>

    </robot>

You can include this file in other files using:

.. code-block:: xml

    <xacro:include filename="example_materials.xacro"/>

.. note::

    When including multiple files:

    .. code-block:: xml

        <xacro:include filename="file1.xacro"/>
        <xacro:include filename="file2.xacro"/>

    The second file can use variables or materials defined in the first file because it is included beforehand. You do not need to re-include *file1.xacro* in *file2.xacro*.

7) *example.urdf.xacro*

Below is the final version of the improved URDF file:

.. code-block:: xml

    <?xml version="1.0"?>
    <robot name="example" xmlns:xacro="http://wwww.ros.org/wiki/xacro">

        <xacro:include filename="example_materials.xacro"/>

        <xacro:property name="base_link_length" value="0.2"/>
        <xacro:property name="base_link_width" value="0.3"/>
        <xacro:property name="base_link_height" value="0.6"/>
        <xacro:property name="second_link_length" value="0.8"/>
        <xacro:property name="second_link_radius" value="0.05"/>

        <xacro:macro name="box" params="length width height">
            <box size="${length} ${width} ${height}"/>
        </xacro:macro>

        <link name="base_link">
            <visual>    
                <origin xyz="0 0 ${base_link_height / 2.0}"  rpy="0 0 0"/>
                <geometry>
                    <xacro:box length="${base_link_length}" width="${base_link_width}" height="${base_link_height}"/>
                </geometry>
                <material name="green"/>
            </visual>
        </link>

        <link name="second_link">
            <visual>
                <origin xyz="${second_link_length / 2.0} 0 0" rpy="0 ${pi / 2.0} 0"/>
                <geometry>
                    <cylinder length="${second_link_length}" radius="${second_link_radius}"/>
                </geometry>
                <material name="blue"/>
            </visual>
        </link>

        <joint name="second_link_joint" type="fixed">
            <parent link="base_link"/>
            <child link="second_link"/>
            <origin xyz="${base_link_length / 2.0} 0 ${base_link_height / 2.0}" rpy="0 0 0"/>
        </joint>

    </robot>

.. note::

    Notice that the file no longer contains hardcoded values. Instead, five variables are used to define the links and joint accurately. While using a macro to define a single box may be excessive here, it serves to demonstrate how macros work.

Thymio - Step 2
~~~~~~~~~~~~~~~

Let’s put this knowledge into practice. The goal is to enhance the previous URDF by utilizing Xacro's features. Follow these steps:

1. Use the pi constant where needed

2. Define variables and replace hardcoded values

3. Create a macro for the wheel links and reuse it for both wheels

4. Split the URDF into three files: *materials.xacro*, *thymio_chassis.xacro*, and *thymio.urdf.xacro*

Additionally, remember to apply mathematical operations wherever possible.

Once again, refer to the drawing below for the key dimensions:

.. image:: img/thymio_spec.png
    :align: center
    :width: 60%

.. |spacer| raw:: html

    <div style="margin-top: 5px;"></div>

|spacer|

.. admonition:: Hints

  .. toggle::

    *  Eight variables are sufficient to define all links and joints: ``base_length``, ``base_width``, ``base_height``, ``ground_clearance``, ``caster_wheel_radius``, ``wheel_radius``, ``wheel_width``, ``wheel_offset``
    * Some variables can depend on others
    * Position the caster wheel and wheels relative to the base's length and width
    * The final visual, with the ``base_length`` increased and the ``base_width`` reduced by a factor of two, should appear as follows:

    .. image:: img/thymio_xacro.png
        :align: center
        :width: 60%


Parameters Overview
-------------------

Let's take a break from URDFs for a moment and explore another essential concept in ROS2: **parameters**. Parameters are configurable values that allow to reuse the same node with differents settings.

To understand their importance, let’s revisit an example from the previous session: the *Heat Index Monitoring System*. We used a temperature sensor and a humidity sensor to calculate the heat index. Now, imagine we want to extend this setup by adding a second temperature sensor, but with different settings, such as a unique publish frequency.

What happens if we try to achieve this without parameters? We might end up duplicating the existing node just to adjust the frequency. This approach quickly becomes inefficient and difficult to manage as the system grows in complexity.

Fortunately, ROS2 parameters provide an elegant solution. They let us configure settings, like the publish frequency, directly from the run command without modifying or duplicating the node’s code. A parameter passed as an argument dynamically updates a variable in the node, enabling efficient customization.

To summarize, ROS2 parameters enable:

* **Customization**: Define robot-specific configurations (e.g. sensor settings)
* **Flexibility**: Adjust node behavior without modifying or rebuilding the code
* **Efficiency**: Reuse the same node with different parameter values

Let’s see how parameters work in practice by modifying the first publisher node we created in session 1. We will define the publish frequency as a parameter, allowing us to change its value directly when running the node from the terminal.

1. Open the file

Open *publisher.py* in the *ros2_basics_pkg* package.

2. Modify the publisher

Replace the contents of *publisher.py* with the following code:

.. code-block:: python

    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import String

    class MinimalPublisher(Node):

        def __init__(self):
            super().__init__('minimal_publisher')
            self.publisher_ = self.create_publisher(String, 'topic', 10)
            self.declare_parameter("publish_frequency", 1.0)
            self.publish_frequency_ = self.get_parameter("publish_frequency").value
            self.timer = self.create_timer(1.0 / self.publish_frequency_, self.timer_callback)
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

**Question:** What are the essential steps involved in working with a parameter?

3. Build the package

|spacer|

4. Test the publisher with different frequencies

Run the node and set the publish frequency using the following command:

.. code-block:: bash

    ros2 run ros2_basics_pkg publisher_node --ros-args -p publish_frequency:=4.0

**Question**: What happens if no parameter value is provided during execution? Why?

.. tip::

    You can verify the frequency at which messages are published using the following command: ``ros2 topic hz <topic_name>``.

.. 2. Declare the parameter

.. A parameter is declared with a name and a default value.

.. .. code-block:: python

..     self.declare_parameter("publish_frequency", 1.0)

.. .. note::

..     If no value is provided when the node is launched, the default value will be used. The default value also determines the parameter’s type. In this case, ``publish_frequency`` is a double.

.. 3. Retrieve the parameter value

.. Use the value provided during the run command or the predefined default value.

.. .. code-block:: python

..     self.publish_frequency_ = self.get_parameter("publish_frequency").value

.. 4. Use the parameter

.. The retrieved value can then be used like any other variable.

.. .. code-block:: python

..     self.timer = self.create_timer(1.0 / self.publish_frequency_, self.timer_callback)

.. 5. Final *publisher.py* 

.. The updated publisher node now looks like this:

.. .. code-block:: python

..     import rclpy
..     from rclpy.node import Node

..     from std_msgs.msg import String

..     class MinimalPublisher(Node):

..         def __init__(self):
..             super().__init__('minimal_publisher')
..             self.publisher_ = self.create_publisher(String, 'topic', 10)
..             self.declare_parameter("publish_frequency", 1.0)
..             self.publish_frequency_ = self.get_parameter("publish_frequency").value
..             self.timer = self.create_timer(1.0 / self.publish_frequency_, self.timer_callback)
..             self.i = 0

..         def timer_callback(self):
..             msg = String()
..             msg.data = 'Hello World: %d' % self.i
..             self.publisher_.publish(msg)
..             self.get_logger().info('Publishing: "%s"' % msg.data)
..             self.i += 1

..     def main(args=None):
..         rclpy.init(args=args)
..         minimal_publisher = MinimalPublisher()
..         rclpy.spin(minimal_publisher)
..         minimal_publisher.destroy_node()
..         rclpy.shutdown()

..     if __name__ == '__main__':
..         main()

Launch Files Overview
---------------------

Gazebo Overview
---------------

Complete URDF - Collision & Inertial
------------------------------------

Gazebo Plugins
--------------

Gazebo Worlds
-------------