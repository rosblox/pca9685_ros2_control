:github_url: https://github.com/ros-controls/ros2_control_demos/blob/|github_branch|/example_2/doc/userdoc.rst

.. _pca9685_hardware_interface_userdoc:

*********
Pca9685
*********

*Pca9685*, or ''Differential Mobile Robot'', is a simple mobile base with differential drive.
The robot is basically a box moving according to differential drive kinematics.

For *example_2*, the hardware interface plugin is implemented having only one interface.

* The communication is done using proprietary API to communicate with the robot control box.
* Data for all joints is exchanged at once.

The *Pca9685* URDF files can be found in ``description/urdf`` folder.

Tutorial steps
--------------------------

1. To check that *Pca9685* description is working properly use following launch commands

   .. code-block:: shell

    ros2 launch pca9685_hardware_interface view_robot.launch.py

   .. warning::
    Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
    This happens because ``joint_state_publisher_gui`` node need some time to start.

   .. image:: pca9685.png
    :width: 400
    :alt: Differential Mobile Robot

2. To start *Pca9685* example open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch pca9685_hardware_interface pca9685.launch.py

   The launch file loads and starts the robot hardware, controllers and opens *RViz*.
   In the starting terminal you will see a lot of output from the hardware implementation showing its internal states.
   This excessive printing is only added for demonstration. In general, printing to the terminal should be avoided as much as possible in a hardware interface implementation.

   If you can see an orange box in *RViz* everything has started properly.
   Still, to be sure, let's introspect the control system before moving *Pca9685*.

3. Check if the hardware interface loaded properly, by opening another terminal and executing

   .. code-block:: shell

    ros2 control list_hardware_interfaces

   You should get

   .. code-block:: shell

    command interfaces
          left_wheel_joint/velocity [available] [claimed]
          right_wheel_joint/velocity [available] [claimed]
    state interfaces
          left_wheel_joint/position
          left_wheel_joint/velocity
          right_wheel_joint/position
          right_wheel_joint/velocity

   The ``[claimed]`` marker on command interfaces means that a controller has access to command *Pca9685*.

4. Check if controllers are running

   .. code-block:: shell

    ros2 control list_controllers

   You should get

   .. code-block:: shell

    pca9685_base_controller[diff_drive_controller/DiffDriveController] active
    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active

5. If everything is fine, now you can send a command to *Diff Drive Controller* using ROS 2 CLI interface:

   .. code-block:: shell

    ros2 topic pub --rate 30 /pca9685_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
      x: 0.7
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 1.0"

   You should now see an orange box circling in *RViz*.
   Also, you should see changing states in the terminal where launch file is started.

   .. code-block:: shell

    [Pca9685SystemHardware]: Got command 43.33333 for 'left_wheel_joint'!
    [Pca9685SystemHardware]: Got command 50.00000 for 'right_wheel_joint'!

Files used for this demos
--------------------------

* Launch file: `pca9685.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/bringup/launch/pca9685.launch.py>`__
* Controllers yaml: `pca9685_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/bringup/config/pca9685_controllers.yaml>`__
* URDF file: `pca9685.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/description/urdf/pca9685.urdf.xacro>`__

  * Description: `pca9685_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/description/urdf/pca9685_description.urdf.xacro>`__
  * ``ros2_control`` tag: `pca9685.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/description/ros2_control/pca9685.ros2_control.xacro>`__

* RViz configuration: `pca9685.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/description/rviz/pca9685.rviz>`__

* Hardware interface plugin: `PCA9685_SYSTEM.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/hardware/PCA9685_SYSTEM.cpp>`__


Controllers from this demo
--------------------------

* ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
* ``Diff Drive Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/diff_drive_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html>`__
