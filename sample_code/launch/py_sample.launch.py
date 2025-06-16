from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare(package='sample_code'), 'rviz', 'rviz.rviz']
    )

    py_sample_node = Node(
        package='sample_code',
        executable='py_sample.py',
        name='py_sample',
        output='screen',
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        py_sample_node,
        rviz2_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=py_sample_node,
                on_exit=[
                    LogInfo(msg='py_sample_node exited'),
                    EmitEvent(event=Shutdown(reason='Closed'))
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=rviz2_node,
                on_exit=[
                    LogInfo(msg='rviz2_node exited'),
                    EmitEvent(event=Shutdown(reason='Closed'))
                ]
            )
        )
    ])
