# ros2_mecademic

An unofficial ROS2 (eloquent) interface for the Mecademic robots

`ros2 launch ros2_mecademic_bringup dummy_robot_bringup.launch.py`

In Rviz2, select `/meca_500_description`

<!-- Publish a command, `at` or `away`, on the `/meca_500_sp_to_interfacer`

See where the robot is on the `/meca_500_interfacer_to_sp` topic -->

TODO:

    1.  Add actual msgs to communicate instead of strings
    2.  Include robot speed control correctly (no flickering)
    3.  Maybe, try MoveIt2 on this example?
    4.  Finalize pose saver