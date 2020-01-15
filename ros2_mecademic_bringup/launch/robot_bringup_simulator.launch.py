import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, actions
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
     urdf1 = os.path.join(get_package_share_directory('ros2_mecademic_description'),
                        'urdf', 'meca_500_r3.urdf')
     rviz_config_file = os.path.join(get_package_share_directory('ros2_mecademic_bringup'),
                        'launch', 'meca_500_r3.rviz')

     return LaunchDescription([
          Node(package='ros2_mecademic_state_publisher', node_executable='ros2_mecademic_state_publisher',
               output='screen', arguments=[urdf1]),
          Node(package='ros2_mecademic_simulator', node_executable='ros2_mecademic_simulator',
               output='screen'), 
          Node(package='ros2_mecademic_gui', node_executable='ros2_mecademic_gui',
               output='screen'),
          Node(package='ros2_mecademic_utilities', node_executable='ros2_mecademic_utilities',
               output='screen'),
          Node(package='rviz2', node_executable='rviz2', arguments=['-d', rviz_config_file],
               output='screen')
     ])