import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  
    rviz_config = os.path.join(
        get_package_share_directory('bin_packing'),
        'config',
        'packing.rviz'
    )
    
    packing_config = os.path.join(
        get_package_share_directory('bin_packing'),
        'config',
        'param.yaml'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
        
    packing_node = Node(
            package='bin_packing',
            executable='packing_node',
            name='packing_node',
            parameters=[packing_config],
            output='screen',
    )
  
    return LaunchDescription([
        rviz_node,
        packing_node,
    ])