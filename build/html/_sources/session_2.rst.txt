Session 2
=========

Introduction
------------

Welcome to session 2 of our ROS2 Basics labs! Today, we will be exploring the visualization and simulation tools integrated with ROS2. In this session, we will dive into Rviz, Transforms (TFs), URDF, Xacro, parameters, launch files, and Gazebo, building the essential skills for designing and simulating robotic systems.

Our main goal for today is to create a model of the Thymio robot that is fully integrated with ROS2 and capable of smooth operation in Gazebo simulation. We will guide you step-by-step through the essential elements needed to achieve this. You will begin with simple examples to introduce each concept and gradually apply this knowledge to build the Thymio model. Now, let's get to work!


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

Rviz also allows you to interact with tools like the *Joint State Publisher* (the second pop-up window), a GUI that lets you manipulate the robot’s joints. This enables you to see how joint movements affect the robot’s structure.

A key question to consider here is: **How does ROS2 determine the positions and movements of the different links relative to one another over time?**

The answer lies in **Transform Frames (TFs)**. These frames represent the spatial relationships (positions and orientations) between the robot’s parts and its environment. TFs enable ROS2 to continuously track how each part of the robot moves in relation to others. By maintaining structured relationships between frames, TFs play a crucial role in various robotic tasks.

Each frame has three axes: x (red), y (green), and z (blue), representing its orientation in 3D space. If you uncheck the ``RobotModel`` in Rviz, you can see that the TFs form a tree-like structure, showing how the robot’s rigid parts are connected.

To better visualize the TF hierarchy, you can use the *view_frames* tool provided by the *tf2_tools* package. Open a terminal and run:

.. code-block:: bash

    cd ~/ros2_basics_ws/
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

Links are the rigid bodies of a robot. They can be described using one of the four types of geometry: **Boxes**, **Cylinders**, **Spheres**, **Meshes**.

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

    .. code-block:: bash

        colcon build --packages-select thymio_description --symlink-install
        source install/setup.bash

    .. warning::

        This command is useful when working with URDFs as it allows you to progressively verify your progress. However, remember to rebuild the package whenever you add a new file.

    Now, we are ready to visualize the result. Use the *urdf_tutorial* package to launch the URDF in Rviz. Later, we will explore how to achieve this without relying on this package.

    .. code-block:: bash

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

    Colors are defined using the ``<material>`` tag. A common practice is to define colors at the top of the file and reference them by name in the ``<visual>`` tag. The color attributes include four arguments: **rgb** for red, green, and blue, and **a** for transparency.

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

    |spacer|

    Our goal now is to replicate this: 

    .. image:: img/urdf_example_final.png
        :align: center
        :width: 50%

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

    .. admonition:: Question
        
        Which origin setting is most critical for ensuring that your robot's movements and positions are accurately represented in ROS2 simulations?

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

.. admonition:: Thymio - Step 1

    As mentioned in the introduction, today's goal is to create a Thymio model that works well in simulation. In addition to building the model, you will set up custom methods to launch applications like Rviz and Gazebo, ensuring they function correctly with your robot. The task is divided into 9 steps, and the journey begins now with your first challenge: creating the visual representation of the Thymio using the tools just introduced.

    Start by creating a new file named *thymio.urdf* in the */urdf/thymio* directory. Use the provided specifications to guide you through the process, and remember to consistently use Rviz to visualize your progress.

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

    |spacer|

    .. admonition:: Hints

        .. toggle::

            * The ground clearance information should be sufficient to define all the heights  
            * Carefully consider where the TFs should be positioned (this is crucial!)  
            * Use the following command to visualize the model in Rviz:

            .. code-block:: bash

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

.. note::

    As a result, the new display command is:

    .. code-block:: bash

        ros2 launch urdf_tutorial display.launch.py model:=/home/ubuntu/ros2_basics_ws/install/thymio_description/share/thymio_description/urdf/example/example.urdf.xacro

2) Xacro compatibility

To enable the use of xacro in our file, we need to adjust the ``<robot>`` tag as follow:

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

.. admonition:: Thymio - Step 2

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

.. admonition:: Question
    
    What happens if no parameter value is provided during execution? Why?

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

In this section, we shift focus back to the visualization aspects of ROS2, closing the parenthesis on parameters. So far, we have relied on a convenient command from the *urdf_tutorial* package to visualize URDF files in Rviz. But how exactly does this command work? Let’s delve into the concept of launch files to uncover the mechanics behind it.

First, let’s recall the command we used previously to display a URDF in Rviz:

.. code-block:: bash

    ros2 launch urdf_tutorial display.launch.py model:=<path_to_urdf>

You might have been wondering, what does the *launch* command do? Simply put, it runs a **launch file**. A launch file is a configuration file that allows you to start multiple nodes simultaneously, often with specific parameters or remapping. This is especially useful when managing complex setups, as launching multiple nodes manually from different terminals can quickly become difficult to manage.

Launch files provide a structured way to define:

* Which nodes to run
* Node-specific parameters
* Topic or service remappings

Launch files can be written in **Python**, **XML**, or **YAML**. For simplicity and conciseness, we will use XML in this course.

Let’s revisit an example from session 1 to better understand launch files. This time, instead of manually starting each node, we will use a launch file to simultaneously start four nodes with their appropriate configurations.

.. image:: img/task2.png
    :align: center
    :width: 60%

|spacer|

1. Create the launch file

.. code-block:: bash

    cd ~/ros2_basics_ws/src/thymio_description/launch/
    touch example.launch.xml

2. Add content to the file

.. code-block:: xml

    <launch>
        <node pkg="demo_nodes_py" exec="talker" name="stress">
            <remap from="chatter" to="exams"/>
        </node>

        <node pkg="demo_nodes_py" exec="talker" name="BA1_students">
            <remap from="chatter" to="exams"/>
        </node>

        <node pkg="demo_nodes_py" exec="listener" name="BA2">
            <remap from="chatter" to="exams"/>
        </node>

        <node pkg="demo_nodes_py" exec="listener" name="MAN">
            <remap from="chatter" to="exams"/>
        </node>
    </launch>

.. admonition:: Question
    
    What are the essential elements of a launch file?

3. Build the package

|spacer|

4. Launch the launch file

.. code-block:: bash

    ros2 launch thymio_description example.launch.xml 

5. Visualize the result

.. code-block:: bash

    rqt_graph

.. note:: 

    Using a launch file, you have successfully started multiple nodes with a single command. Additionally, remapping topics has become significantly more convenient.

Now that you know that a launch file start multiple nodes at the same time, let's discover what the *display.launch.py* actually launch.

1. Run the launch command with the Thymio model

.. code-block:: bash
    
    ros2 launch urdf_tutorial display.launch.py model:=/home/ubuntu/ros2_basics_ws/install/thymio_description/share/thymio_description/urdf/thymio/thymio.urdf.xacro

2. Visualize the graph

.. code-block:: bash

    rqt_graph 

3. Configure the *Node Graph*

.. image:: img/rqt_config.png
    :align: center
    :width: 70%

|spacer|

.. As you can see, we have two main nodes that are running: ``joint_state_publisher`` and ``robot_state_publisher``. The ``robot_state_publisher`` is the node that is in charge of keeping track of the TFs in ROS2. All its needs is to know is where the joints are placed and this is provided by the ``joint_state_publisher`` node that publishes on the ``joint_states`` topic. Here, the ``joint_state_publisher`` is GUI interface that allows us to play with the movable joints and provides the virtual positioning of the joints. In real life, the joint information would be given by sensors such as encoders.
.. Previously, we mentioned that we needed to provide the URDF to ROS2 so that it can correctly manage the TFs. Indeed, this is a requirement for the ``robot_state_publisher`` to work. But, where is the URDF located? This is time to exploit our knowledge on parameters! When we use the launch command we always provide the path to our URDF file which is a parameter that we give to this ``robot_state_publisher`` node. Are you not convinced? Ok, let's prove it then:

Looking at the *rqt_graph*, we see two main nodes interacting: ``joint_state_publisher`` and ``robot_state_publisher``. The ``robot_state_publisher`` handles TFs in ROS2 by relying on joint information published by the ``joint_state_publisher``. In this case, the ``joint_state_publisher`` is a GUI tool that lets us adjust joint positions virtually. In a real-world scenario, joint positions would be published by hardware sensors, such as encoders.

For the ``robot_state_publisher`` to work, it needs the URDF, which defines the robot's structure and joint placements. This URDF file is passed as a parameter during the launch process. But where exactly can we find it? Let’s explore this:

1. List the different nodes

.. code-block:: bash

    ros2 node list

2. List the parameter of the ``robot_state_publisher`` node

.. code-block:: bash

    ros2 param list /robot_state_publisher 

3. Check the content of the *robot_description* parameter

.. code-block:: bash

    ros2 param get /robot_state_publisher robot_description 

Now that we have located the parameter containing the Thymio robot’s URDF, let’s take a closer look. This parameter holds the complete description of the robot, which was originally split across three files. Using the *xacro* tool, these files were combined into a single, unified URDF. You can confirm this in the terminal, where the file header states: *This document was autogenerated by xacro*.

To summarize, let’s refer to the following image for a visual representation:

.. figure:: img/robot_description.png
    :align: center
    :width: 80%

    `Describing robots with URDF (Articulated Robotics) <https://articulatedrobotics.xyz/tutorials/ready-for-ros/urdf>`_

.. In conclusion, the ``robot_state_publisher`` update the robot model and TFs over time as long as it has been provided with the URDF file as a parameter and as long as it receives information on the joint positions. The ``joint_state_publisher`` gives the virtual position of the joints which is replace by sensors in real-life applications.

Here’s a quick recap of the roles of the two nodes:

* ``robot_state_publisher``:

    * Updates the robot model and TFs in real-time
    * Requires the URDF file as a parameter to define the robot's structure
    * Relies on joint position data to reflect changes in the robot's state

* ``joint_state_publisher``:

    * Provides virtual joint positions in simulation
    * Replaced by hardware sensors, such as encoders, in real-world applications

Now that we understand the key components behind this launch process, let’s run each node individually in separate terminals to see how they work.

1. Run the ``robot_state_publisher`` node

From the previous explanation, we need to launch the ``robot_state_publisher`` node and provide the URDF file as a parameter. Additionally, we must use the *xacro* tool to combine the Xacro files into a single URDF file.

.. code-block:: bash

    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/ubuntu/ros2_basics_ws/install/thymio_description/share/thymio_description/urdf/thymio/thymio.urdf.xacro)"

2. Run the ``joint_state_publisher`` node

.. code-block:: bash

    ros2 run joint_state_publisher_gui joint_state_publisher_gui 

3. Run *Rviz*

.. code-block:: bash

    ros2 run rviz2 rviz2

At this stage, nothing is visible in Rviz. To proceed, you need to configure the interface with the required display settings. Start by adding the ``RobotModel`` and ``TF`` plugins, and then adjust their options as follows:

.. image:: img/Rviz_config.png
    :align: center
    :width: 40%

|spacer|

.. important::

    Save the configuration in the *rviz* directory of the *thymio_description* package. In Rviz, navigate to *File > Save Config As* and select the appropriate location. This saves your current setup exactly as it appears on your screen. In the next task, you will add this configuration to a launch file, making it convenient to avoid reconfiguring Rviz each time.

    The terminal command would look like this: 

    .. code-block:: bash

        ros2 run rviz2 rviz2 -d "/home/ubuntu/ros2_basics_ws/install/thymio_description/share/thymio_description/rviz/<config_name>.rviz"

4. Run *rqt_graph*

Finally, notice that the result is identical to what we achieved using the *display.launch.py* file from the *urdf_tutorial* package.

.. admonition:: Thymio - Step 3

    The next challenge is to create a launch file (*thymio_display.launch.xml*) that starts the following three executables simultaneously: ``robot_state_publisher``, ``joint_state_publisher_gui``, and ``rviz2``. Ensure that Rviz is launched with the configuration file you previously saved to maintain your custom display settings.

    To assist you, a generic example is provided below, which should give you the tools necessary to complete the task. Additionally, it is highly recommended to review the terminal commands previously used to start each executable individually, as these will help you structure your launch file.

    .. code-block:: xml

        <launch>
            <!-- Define an argument for a file path -->
            <arg name="file_path" default="$(find-pkg-share <package_name>)/<path_to_file>"/>

            <!-- Use the argument to parse a file with a command and set it as a parameter -->
            <node pkg="<package_name>" exec="<executable_name>"> 
                <param name="<parameter_name>" value="$(command 'tool_name $(var file_path)')"/>
            </node>

            <!-- Use the argument as a command-line argument -->
            <node pkg="<package_name>" exec="<executable_name>" args="-a $(var file_path)"/>
        </launch>

After successfully creating and testing your launch file, compare it with the Python version provided below.

.. admonition:: Python Launch File

  .. toggle::

    Python launch files may be slightly more complex to write, but they provide greater flexibility.

    .. code-block:: python

        import os
        from launch_ros.actions import Node
        from launch import LaunchDescription
        from launch.substitutions import Command
        from launch_ros.parameter_descriptions import ParameterValue
        from ament_index_python.packages import get_package_share_path

        def generate_launch_description():

            urdf_path = os.path.join(get_package_share_path('thymio_description'),
                                     'urdf', 'thymio.urdf.xacro')
            
            rviz_config_path = os.path.join(get_package_share_path('thymio_description'),
                                            'rviz', 'rviz_config.rviz')

            robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

            robot_state_publisher_node = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{'robot_description': robot_description}]
            )

            joint_state_publisher_gui_node = Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui"
            )

            rviz2_node = Node(
                package="rviz2",
                executable="rviz2",
                arguments = ["-d", rviz_config_path]
            )

            return LaunchDescription([
                robot_state_publisher_node,
                joint_state_publisher_gui_node,
                rviz2_node
            ])


Gazebo Overview
---------------

Now, let's take the next step and introduce simulation into our workflow using **Gazebo**. Gazebo is a **physics-based simulation tool** that integrates with ROS2 to provide:

* A virtual environment for testing robot behaviors
* Accurate simulations of physical interactions and sensor outputs

Simulation is an essential part of robotics development for several reasons:

* **Algorithm testing**: Optimizes control algorithms in a repeatable, controlled environment
* **Physical interactions**: Models collisions, dynamics, gravity, inertia, and sensor noise
* **Sensors and actuators simulation**: Provides sensor data and enables actuator control

Gazebo is an independent tool and not a native part of the ROS2 environment. However, it integrates with ROS2 using the *gazebo_ros* package, which acts as a bridge between the two. This integration is made possible through various Gazebo plugins that allow interaction with the ROS2 ecosystem and simulation of robot hardware, including actuators and sensors.

.. note::

    To clarify the differences between **Rviz** and **Gazebo**, here is a summary table:

    .. raw:: html

        <div class="table-centered">

    +------------------------+-----------------------------+----------------------------------+
    | **Feature**            | **Rviz**                    | **Gazebo**                       |
    +========================+=============================+==================================+
    | **Purpose**            | 3D visualization tool for   | Simulation tool that models      |
    |                        |                             |                                  |
    |                        | monitoring robot state      | real-world physical properties   |
    +------------------------+-----------------------------+----------------------------------+
    | **Control**            | No control capabilities     | Provides control functionalities |
    |                        |                             |                                  |
    |                        | (purely visualization)      |                                  |
    +------------------------+-----------------------------+----------------------------------+
    | **Use Case**           | Debugging and analyzing     | Simulating and testing robots in |
    |                        |                             |                                  |
    |                        | robot data                  | a realistic environment          |
    +------------------------+-----------------------------+----------------------------------+

.. Let's move to the next step and try to add simulation in our journey. For this we will use **Gazebo**. Gazebo is a physics-based simulation tool that operates with ROS2 for:

.. * Testing robot behaviors in a virtual environment
.. * Simulating physical interactions and sensor data

.. Like any other simulation tool, Gazebo is essential as it:

.. * Enables testing and refining robot control algorithms in a repeatable, controlled environment before real-world deployment
.. * Simulates physical interactions (collisions, dynamics), sensor inputs (cameras, LiDAR), and robot control systems
.. * Incorporates real-world factors like gravity, inertia, and sensor noise

.. It is also good to mention that Gazebo is an independent tool that is not part of the ROS2 environment. However, its use is possible thanks to the *gazebo_ros* package that makes the bridge between the two. Therefore, there exists some Gazebo plugins that can be used to interact with the ROS2 ecosystem and that enable to simulate the robot hardware (actuators, sensors).

.. To avoid any confusion between Rviz and Gazebo, here is a table that summarizes their main features:


Complete URDF - Collision & Inertial
------------------------------------

So far, we have been focusing exclusively on the visual representation of the Thymio robot. To make the model functional in a simulation, the URDF must be updated to include collision and physical properties. These additions will enable the robot to interact realistically with the virtual environment. Let's begin by incorporating the collision properties:

1. Launch our simple example in Rviz

.. code-block:: bash

    ros2 launch urdf_tutorial display.launch.py model:=/home/ubuntu/ros2_basics_ws/install/thymio_description/share/thymio_description/urdf/example/example.urdf.xacro

2. Adjust the Rviz configuration

Under the ``RobotModel`` options, uncheck *Visual Enabled* and check *Collision Enabled*. You will see that nothing appear. In fact, this is normal, we have not defined the collision properties yet.

3. Update *example.urdf.xacro*

To add collision properties to the links in the URDF, you need to include ``<collision>`` tags. These are similar to ``<visual>`` tags: both require an origin and a geometry. However, ``<collision>`` tags do **not** require a material definition.

Here is the revised version of the code with collision properties included for the links:

.. code-block:: xml

    <link name="base_link">
        <visual>    
            <origin xyz="0 0 ${base_link_height / 2.0}"  rpy="0 0 0"/>
            <geometry>
                <xacro:box length="${base_link_length}" width="${base_link_width}" height="${base_link_height}"/>
            </geometry>
            <material name="green"/>
        </visual>
        <collision>    
            <origin xyz="0 0 ${base_link_height / 2.0}"  rpy="0 0 0"/>
            <geometry>
                <xacro:box length="${base_link_length}" width="${base_link_width}" height="${base_link_height}"/>
            </geometry>
        </collision>
    </link>

    <link name="second_link">
        <visual>
            <origin xyz="${second_link_length / 2.0} 0 0" rpy="0 ${pi / 2.0} 0"/>
            <geometry>
                <cylinder length="${second_link_length}" radius="${second_link_radius}"/>
            </geometry>
            <material name="blue"/>
        </visual>
        <collision>
            <origin xyz="${second_link_length / 2.0} 0 0" rpy="0 ${pi / 2.0} 0"/>
            <geometry>
                <cylinder length="${second_link_length}" radius="${second_link_radius}"/>
            </geometry>
        </collision>
    </link>

4. Verify the result in Rviz

.. admonition:: Question

    Is the geometry defined in the ``<collision>`` tag always identical to the visual geometry? You can base your reflection on the following example. Think about why they might differ and how this affects simulation and robot interactions.

    .. image:: img/BlockBuster.png
        :align: center
        :width: 65%
    
    |spacer|

Now, let's focus on defining the physical properties of the model by adding the ``<inertial>`` tags. These tags play a crucial role in accurately simulating the robot's motion by specifying properties such as mass and moments of inertia. This ensures the model responds realistically to forces like gravity and other dynamics.

As you may recall from your physics courses, the formulas for the moment of inertia tensor are well-established for basic geometric shapes. These tensors are uniquely determined by the object's dimensions and mass. Let’s explore how to incorporate these inertia properties:

1. Create an *example_inertia.xacro* file

In the */urdf/example* directory, create a file named *example_inertia.xacro* to define reusable macros for the inertia properties of our basic shapes. 

2. Add the inertia macros to the file

Like the other tags we have encountered, the ``<inertial>`` tag requires an origin to be defined. In addition, it needs a mass and an inertia matrix. Since this matrix is symmetric, only 6 of its 9 components need to be specified. You can consult the formulas for defining inertia matrices in the corresponding Wikipedia webpage: `List of 3D inertia tensors <https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors>`_.

.. code-block:: xml

    <?xml version="1.0"?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro">

        <xacro:macro name="box_inertia" params="m l w h xyz rpy">
            <inertial>
                <origin xyz="${xyz}" rpy="${rpy}"/>
                <mass value="${m}"/>
                <inertia ixx="${(m/12) * (h*h + l*l)}" ixy="0" ixz="0"
                        iyy="${(m/12) * (w*w + l*l)}" iyz="0"
                        izz="${(m/12) * (w*w + h*h)}"/>
            </inertial>
        </xacro:macro>

        <xacro:macro name="cylinder_inertia" params="m r h xyz rpy">
            <inertial>
                <origin xyz="${xyz}" rpy="${rpy}"/>
                <mass value="${m}" />
                <inertia ixx="${(m/12) * (3*r*r + h*h)}" ixy="0" ixz="0"
                        iyy="${(m/12) * (3*r*r + h*h)}" iyz="0"
                        izz="${(m/2) * (r*r)}"/>
            </inertial>
        </xacro:macro>

    </robot>

3. Adjust *example.urdf.xacro*

To use the defined macros, include the previously created file and invoke them with the desired parameters. 

.. code-block:: xml

    <xacro:include filename="example_inertia.xacro"/>

    <xacro:property name="base_link_mass" value="0.5"/> <!-- Mass in [kg] -->
    <xacro:property name="second_link_mass" value="0.2"/> <!-- Mass in [kg] -->

    <link name="base_link">
        <visual>    
            <origin xyz="0 0 ${base_link_height / 2.0}"  rpy="0 0 0"/>
            <geometry>
                <xacro:box length="${base_link_length}" width="${base_link_width}" height="${base_link_height}"/>
            </geometry>
            <material name="green"/>
        </visual>
        <collision>    
            <origin xyz="0 0 ${base_link_height / 2.0}"  rpy="0 0 0"/>
            <geometry>
                <xacro:box length="${base_link_length}" width="${base_link_width}" height="${base_link_height}"/>
            </geometry>
        </collision>
        <xacro:box_inertia m="${base_link_mass}" l="${base_link_length}" w="${base_link_width}" h="${base_link_height}"
                           xyz="0 0 ${base_link_height / 2.0}"  rpy="0 0 0"/>
    </link>

    <link name="second_link">
        <visual>
            <origin xyz="${second_link_length / 2.0} 0 0" rpy="0 ${pi / 2.0} 0"/>
            <geometry>
                <cylinder length="${second_link_length}" radius="${second_link_radius}"/>
            </geometry>
            <material name="blue"/>
        </visual>
        <collision>
            <origin xyz="${second_link_length / 2.0} 0 0" rpy="0 ${pi / 2.0} 0"/>
            <geometry>
                <cylinder length="${second_link_length}" radius="${second_link_radius}"/>
            </geometry>
        </collision>
        <xacro:cylinder_inertia m="${second_link_mass}" r="${second_link_radius}" h="${second_link_length}" 
                                xyz="${second_link_length / 2.0} 0 0" rpy="0 ${pi / 2.0} 0"/>

.. note::

    If you are interested, you can visualize the result in Rviz (*RobotModel > Mass Properties > Inertia*), but this method is not ideal for intuitively verifying the correctness of our implementation. Instead, we recommend testing the physical behavior directly in Gazebo. However, as we do not yet have all the necessary tools, we will revisit this step later.

.. admonition:: Thymio - Step 4

    The new task is to enhance the current Thymio model by adding ``<collision>`` and ``<inertial>`` tags.

    .. raw:: html

        <div class="table-centered">

    +------------------------+-----------------------------+
    | **Component**          | **Specification**           |
    +========================+=============================+
    | *Total mass*           | *270 g*                     |
    +------------------------+-----------------------------+
    | Wheels                 | 20% of total mass           |
    +------------------------+-----------------------------+
    | Chassis                | 80% of total mass           |
    | |spacer|               | |spacer|                    |
    | ┣━ Base_link           | 95% of chassis mass         |
    |                        |                             |
    | ┗━ Caster wheel        | 5% of chassis mass          |
    +------------------------+-----------------------------+


Spawn Robot in Gazebo
---------------------

Your Thymio robot is now ready for simulation testing. Let’s see how to launch it in Gazebo:

1. Launch *thymio_display.launch.xml*

.. code-block:: bash

    ros2 launch thymio_description thymio_display.launch.xml 

2. Launch *Gazebo*

.. code-block:: bash

    ros2 launch gazebo_ros gazebo.launch.py

3. Spawn the Thymio in Gazebo

.. code-block:: bash

    ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity thymio

.. note::
    
    These commands are provided by the *gazebo_ros* package, which serves as the interface connecting ROS2 and Gazebo.

.. admonition:: Thymio - Step 5

    For convenience, include the two commands in *thymio_display.launch.xml*.

    .. admonition:: Hints

        .. toggle::

            * To include a launch file within another launch file, use the following command:

            .. code-block:: xml

                <include file="$(find-pkg-share <package_name>)/<path_to_file>"/>

            The ``find-pkg-share <package_name>`` command locates packages installed in ``/opt/ros/humble/share``. If you are unsure of a launch file's path, this directory is a good starting point for exploration.

            * To add command-line arguments to your launch file, use the following syntax:

            .. code-block:: xml

                <node pkg="<package_name>" exec="<executable_name>" args="-a arg"/>

After completing this task, return to Gazebo and experiment with the physics to observe the robot's behavior:

* *Translation*: Press ``T``, click the robot, and drag to move or lift it. Try to make it fall.
* *Rotation*: Press ``R``, click the robot, and rotate or tilt it. Observe how it stabilizes.

Once you have observed the physics in action, go back to the URDF file. Comment out the ``<collision>`` tag for the wheel links, save the changes, and relaunch the simulation.

.. admonition:: Question

    What do you observe? How does the absence of a collision property affect the robot's interaction with its environment?

After addressing this question, revisit the URDF file, restore the ``<collision>`` tag, and now comment out the ``<inertial>`` tag for the wheels. Save your changes and relaunch the simulation.

.. admonition:: Question

    How does the behavior differ this time? Why do you think this occurs?

If you paid attention to the simulation results, you may have noticed two problems:

    1. The Thymio moves slightly on its own after spawning
    2. Colors are missing in Gazebo

Let’s tackle these one by one. The unexpected motion occurs because the simplified Thymio model lacks accurate inertia properties and precise mass values. To resolve this, we will adjust the dynamics of the wheel joints and reduce the friction of the caster wheel.

    1. Modify the wheel joint dynamics

    For each wheel joint, add the following line:

    .. code-block:: xml

        <dynamics damping="0.1" friction="0.2"/>

    2. Create a *gazebo.xacro* 

    Gazebo provides specific ``<gazebo>`` tags to define simulation properties. To keep things organized, create a new Xacro file (*gazebo.xacro*) where we will add all Gazebo-specific properties.

    3. Reduce the caster wheel friction

    The caster wheel in the current robot model adds too much friction and drags against the ground. To fix this, reduce the friction coefficients using the following values:

    .. code-block:: xml

        <gazebo reference="caster_wheel_link">
            <mu1 value="0.31" />
            <mu2 value="0.31" />
        </gazebo>

    After making these adjustments, the Thymio should remain motionless upon spawning. Test it to confirm.

To address the missing colors, we can use ``<gazebo>`` tags to define materials for the links. For example, to apply a green color to a link named *example_link*:

.. code-block:: xml

    <gazebo reference="example_link">
        <material>Gazebo/Green</material>
    </gazebo>

.. admonition:: Thymio - Step 6

    Update the colors in *gazebo.xacro* to achieve the desired visual appearance in Gazebo.
    

Gazebo Plugins
--------------

Your Thymio is almost ready for simulation. The next step is to add control capabilities so the robot can move, and optionally, proximity sensors to detect obstacles. This chapter will guide you through these steps. Gazebo provides a range of plugins that simplify this process, as seen in the `gazebo_plugins <https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins/include/gazebo_plugins>`_ repository for ROS2.

To begin, let’s focus on control. The Thymio is a differential drive robot, so we need a plugin that functions as a differential drive controller. From the repository linked above, you can find the ``gazebo_ros_diff_drive`` plugin, which fulfills this role. The `gazebo_ros_diff_drive.hpp <https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/include/gazebo_plugins/gazebo_ros_diff_drive.hpp>`_ file provides details on how to use this plugin effectively, which we will adapt for our application.

.. admonition:: Thymio - Step 7

    Open the *gazebo.xacro* file and paste the following code:

    .. code-block:: xml

        <gazebo>
            <plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so">
        
            <!-- Update rate in Hz -->
            <update_rate>50</update_rate>
        
            <!-- wheels -->
            <left_joint>???</left_joint>
            <right_joint>???</right_joint>
        
            <!-- kinematics -->
            <wheel_separation>???</wheel_separation>
            <wheel_diameter>???</wheel_diameter>

            <!-- input -->
            <command_topic>cmd_vel</command_topic>
        
            <!-- output -->
            <publish_odom>true</publish_odom>
            <publish_odom_tf>true</publish_odom_tf>
            <publish_wheel_tf>true</publish_wheel_tf>
        
            <odometry_topic>odom</odometry_topic>
            <odometry_frame>odom</odometry_frame>
            <robot_base_frame>???</robot_base_frame>
        
            </plugin>
        </gazebo>

    This snippet includes the essential elements needed for the Thymio. Replace the ``???`` with the appropriate values. Accurately defining these parameters is critical for precise motion control.  

.. note:: 

    The ``gazebo_ros_diff_drive`` plugin takes velocity commands from the ``cmd_vel`` topic and rotates the wheels accordingly to achieve the desired motion. It also updates TFs and computes odometry for the robot.  

Once the plugin configuration is complete, follow these steps to test it:  

    1. Update *thymio_display.launch.xml*

    The plugin handles wheel control and simulates real-life encoders by tracking the wheel joints' positions. This makes the *joint_state_publisher_gui* redundant for wheel joints, so comment it out in the launch file.  

    2. Launch the Gazebo simulation

    |spacer|

    3. Send velocity commands from the terminal

    Open a new terminal and run:

    .. code-block::

        ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

    This command sends a *Twist* message, which controls the robot's velocities. The *Twist* type allows control of three linear velocities and three angular velocities. For a differential drive robot like Thymio, you can only adjust the linear speed in the x-direction and angular speed around the z-axis. 
    
    Experiment with different commands and observe how the robot responds. Additionally, try removing the friction coefficients added to the caster wheel link to explore how this impacts the Thymio's behavior in the simulation. 

.. error::

    Despite addressing most motion-related issues, the robot may still behave unexpectedly when moving backward. Specifically, the Thymio tends to deviate and turn instead of maintaining a straight line. Forward and rotational movements are more stable, so it is recommended to prioritize these and avoid using backward motion whenever possible.

The diagram below provides a concise summary of everything covered so far. Review it to ensure you understand the components and their connections. If anything is unclear, do not hesitate to ask questions.

.. figure:: img/gazebo.png
    :align: center
    :width: 90%

    `Understanding control in Gazebo (Articulated Robotics) <https://articulatedrobotics.xyz/tutorials/mobile-robot/concept-design/concept-gazebo>`_

.. admonition:: Break Time

    If this session felt intense, take a moment to relax and have some fun with your Thymio. You can control it using your keyboard by running the following command:

    .. code-block:: bash

        ros2 run teleop_twist_keyboard teleop_twist_keyboard

So far, you have constructed a Thymio model capable of movement using Gazebo's differential drive plugin. However, an essential feature is still missing: **sensors**. 

Typically, adding sensors in a URDF involves the following steps:

    * **Create and position sensor links**: Each sensor requires a dedicated link in the URDF file, accurately placed to match the robot's structure.
    * **Define sensor properties**: Use ``<sensor>`` tags to specify parameters like type, range, and topic names.
    * **Integrate Gazebo plugins**: Add plugins to enable the sensors to function in the simulation environment.

In our case, we want to add proximity sensors to the Thymio, enabling it to detect obstacles.

.. admonition:: Thymio - Step 8

    Adding multiple sensors manually can be a complex and time-consuming process. To simplify this, we provide a pre-configured file that facilitates the integration of proximity sensors into your Thymio model. Just follow the steps below:

    1. :download:`Download <downloads/proximity_sensors.zip>` the configuration file

    Save the provided *proximity_sensors.xacro* file in the */urdf/thymio directory*.

    2. Include the file in your URDF

    Add the file to *thymio.urdf.xacro*, ensuring it is included after *thymio_chassis.xacro* as it references the ``base_link``.

    3. Adjust the ``base_link`` dimensions

    Set the ``base_link`` length to 0.091 (reduced from 0.11) to correctly position the proximity sensors at the front of the Thymio. This adjustment ensures the sensors are properly positioned outside the chassis while maintaining the robot's real-world dimensions.

Now, you are ready to test the updated model. Build the package and launch the enhanced Thymio in Gazebo to observe the result.


Gazebo Worlds - Optional
------------------------

To conclude today's session, let's add the final element needed to achieve a fully functional simulation in ROS2 with Gazebo. So far, we have been working in an empty environment, but to test local avoidance algorithms, it would be beneficial to introduce obstacles into the world. There are two main ways to customize your world:

1. Adding objects

    Open the ``Insert`` panel in Gazebo, where you can find libraries (e.g. http://models.gazebosim.org/) offering various objects to include in your world. When moving an object, hold down ``Shift`` to snap it to the grid for better alignment.

    .. warning::

        When launching Gazebo, it may take some time for the object libraries to load. Be patient, and the list of objects should appear shortly.  

2. Adding walls

    Navigate to *Edit > Building Editor*, which opens a window where you can create custom rooms. The interface consists of two main parts:

    * **Editing interface** (squared sheet): This is where you design your room layout by selecting *Walls* and drawing directly on the grid. You can also add features such as doors and windows by placing them onto the walls.
    * **Preview area**: This section provides a visualization of the room as you design it. You can add textures to the walls directly in this area.

    |spacer|

    Once finished, save your work by going to *File > Save*. Save the room at the default location with your chosen name. Exit the editor via *File > Exit Building Editor*.

    The room you created will now appear as a new object that you can place in your environment. 

After finalizing your environment, ensure the Thymio robot is not part of the saved world. If it is, delete it to avoid having it treated as an object when the world is reopened. Save the world by going to *File > Save World As* and save it as *gazebo_world.world* in the */urdf/world* directory of the *thymio_description* package.

Once saved, build the package and run:

.. code-block:: bash

    ros2 launch gazebo_ros gazebo.launch.py world:=/home/ubuntu/ros2_basics_ws/install/thymio_description/share/thymio_description/worlds/gazebo_world.world

.. note::

    If you open *gazebo_world.world*, you will notice that it is written in SDF (Simulation Description Format), which, like the URDF file, is an XML-based format. The SDF defines Gazebo's simulation environment, specifying objects, lights, physics and environmental parameters.

.. admonition:: Thymio - Step 9

    If you have not created your custom Gazebo world yet, start by doing so. Once completed and saved in the */urdf/world* directory, update the launch file to ensure Thymio spawns directly into the world.  

    .. admonition:: Hints

        .. toggle::

            To pass arguments when including other launch files, use the following structure:  

            .. code-block:: xml

                <include file="$(find-pkg-share <package_name>)/<path_to_file>"/>
                    <arg name="<arg_name>" value="<arg_value>"/>
                </include>

.. note::

    You can also adjust the spawn position of the Thymio using this syntax:  

    .. code-block:: xml

        <node pkg="gazebo_ros" exec="spawn_entity.py"
           args="-topic robot_description -entity thymio 
                 -x 5.0 -y 0.0 -z 2.0
                 -R 0.0 -P 0.0 -Y -1.57"/>