# ros2_mecademic

An unofficial ROS2 (eloquent) interface for the Mecademic robots

`ros2 launch ros2_mecademic_bringup dummy_robot_bringup.launch.py`

Launch Rviz2 and select `/meca_500_description`

Publish a command, `at` or `away`, on the `/meca_500_sp_to_interfacer`

See where the robot is on the `/meca_500_interfacer_to_sp` topic

TODO:

    1.  Add actual msgs to communicate instead of strings
    2.  Robot speed control via msg line in 1.
    3.  Port joint_state_publisher from ROS1 to adjust the robot pose online
    4.  Pose saver as in unification, maybe also add GUI
    5.  Maybe, try MoveIt2 on this example?