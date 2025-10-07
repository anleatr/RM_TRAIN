from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('hik_camera')
    rviz_cfg_path = os.path.join(pkg_share, 'rviz', 'hik.rviz')

    # ********  关键打印  ********
    print('[launch] rviz config path =', rviz_cfg_path)
    print('[launch] file exists ?', os.path.isfile(rviz_cfg_path))
    # ********************************

    hik_node = Node(
        package='hik_camera',
        executable='hik_camera_node',
        name='hik_camera',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg_path]
    )

    return LaunchDescription([hik_node, rviz_node])