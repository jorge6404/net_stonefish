"""Tanque CIRTESU con el BlueROV con la pinza de dos dedos y la red blanda que agarra.

Es net_cirtesu.launch.py con scenario = scenarios/bluerov_gripper_cirtesu_net.scn y net_config = config/net_gripper.yaml.
Acepta los mismos args (simulation_rate, window_width, window_height, damage_seed, rendering_quality, ...), que se
pasan tal cual.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('stonefish_net')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'net_cirtesu.launch.py'])),
            launch_arguments={
                'scenario': PathJoinSubstitution([share, 'scenarios', 'bluerov_gripper_cirtesu_net.scn']),
                'net_config': PathJoinSubstitution([share, 'config', 'net_gripper.yaml']),
            }.items(),
        ),
    ])
