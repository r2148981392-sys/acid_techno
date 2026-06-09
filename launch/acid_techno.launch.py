from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='acid_techno',
            executable='corrected_odom_node',
            name='corrected_odom_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='acid_techno',
            executable='gui_node',
            name='gui_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='acid_techno',
            executable='read_acidity_node',
            name='read_acidity_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='acid_techno',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
