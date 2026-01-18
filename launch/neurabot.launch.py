from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    nav_node = Node(
        package='object_detection',
        executable='nav_autonome',
        name='nav_node',
        output='screen'
    )

    detection_node = Node(
        package='object_detection',
        executable='detection_node',
        name='detection_node',
        output='screen'
    )

    decision_node = Node(
        package='object_detection',
        executable='decision_node',
        name='decision_node',
        output='screen'
    )

    action_node = Node(
        package='object_detection',
        executable='action_node',
        name='action_node',
        output='screen'
    )

    return LaunchDescription([
        nav_node,
        detection_node,
        decision_node,
        action_node
    ])
