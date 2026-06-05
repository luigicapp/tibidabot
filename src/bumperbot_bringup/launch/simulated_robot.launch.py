import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="empty"
    )
    

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_description"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "world_name": LaunchConfiguration("world_name")
        }.items(),
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "launch",
                "joystick_teleop.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    return LaunchDescription([
        world_name_arg,
        gazebo,
        controller,
        joystick,
    ])