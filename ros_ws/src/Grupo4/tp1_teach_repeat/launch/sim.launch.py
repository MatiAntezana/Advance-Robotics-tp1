"""Gazebo Harmonic: bridge sin odom/TF y URDF con raíz base_link."""
import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tp1_teach_repeat.runtime import acquire_simulation_lock


def generate_launch_description():
    acquire_simulation_lock()
    tb = get_package_share_directory('turtlebot3_gazebo')
    share = get_package_share_directory('tp1_teach_repeat')
    gz = get_package_share_directory('ros_gz_sim')
    root = ET.parse(os.path.join(tb, 'urdf', 'turtlebot3_burger.urdf')).getroot()
    # Solo modifica la descripción en memoria. El underlay permanece intacto.
    for element in list(root):
        if element.attrib.get('name') in ('base_footprint', 'base_joint'):
            root.remove(element)
    description = ET.tostring(root, encoding='unicode')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value='-2.0'),
        DeclareLaunchArgument('y', default_value='-0.5'),
        DeclareLaunchArgument('world', default_value=os.path.join(tb, 'worlds', 'turtlebot3_world.world')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb, 'models')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gz_sim.launch.py')),
                                 launch_arguments={'gz_args': ['-r -s -v2 ', LaunchConfiguration('world')]}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gz_sim.launch.py')),
                                 launch_arguments={'gz_args': '-g -v2'}.items(),
                                 condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'burger', '-file', os.path.join(tb, 'models', 'turtlebot3_burger', 'model.sdf'),
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'), '-z', '0.01', '-world', 'default']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml')}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),
    ])
