from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='forward_kinematics',
            namespace='forward_kinematics',
            executable='forward_kinematics_node',
            name='fk'
        ),
        Node(
            package='inverse_kinematics',
            namespace='inverse_kinematics',
            executable='inverse_kinematics_node',
            name='ik'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('open_manipulator_x_controller'),
                    'launch',
                    'open_manipulator_x_controller.launch.py'
                ])
            ])
        )
    ])
 


