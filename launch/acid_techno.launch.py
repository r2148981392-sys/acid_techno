from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='acid_techno',
            executable='gui_node',
            name='gui_node',
            output='screen',
            parameters=[]
        ),
        Node(
            package='acid_techno',
            executable='read_acidity_node',
            name='read_acidity_node',
            output='screen',
            parameters=[]
        ),
        Node(
            package='acid_techno',
            executable='path_find_node',
            name='path_find_node',
            output='screen',
            parameters=[]
        ),
    ])
