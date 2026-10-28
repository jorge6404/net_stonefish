"""Tanque CIRTESU con el BlueROV y la red blanda (stonefish_net_simulator).

Equivale a bluerov_cirtesu.launch.py de bluerov_stonefish con el simulador de la red.
Args: scenario (.scn; por defecto el del tanque con la red), simulation_data (carpeta de las mallas y texturas
del escenario; por defecto la de bluerov_stonefish), net_config (YAML de la red), simulation_rate (200 por
defecto, para la malla de 10 cm de net.yaml), window_width y window_height (1920×1080), damage_seed (semilla
de los agujeros; -1 = la del YAML), rendering_quality (low|medium|high|noshadows).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('stonefish_net')
    scenario = LaunchConfiguration('scenario')
    simulation_data = LaunchConfiguration('simulation_data')
    net_config = LaunchConfiguration('net_config')
    simulation_rate = LaunchConfiguration('simulation_rate')
    damage_seed = LaunchConfiguration('damage_seed')
    rendering_quality = LaunchConfiguration('rendering_quality')

    window_width = LaunchConfiguration('window_width')
    window_height = LaunchConfiguration('window_height')

    simulator = Node(
        package='stonefish_net',
        executable='stonefish_net_simulator',
        namespace='stonefish_ros2',
        name='stonefish_simulator',
        output='screen',
        arguments=[
            simulation_data, scenario, simulation_rate, window_width, window_height, rendering_quality, net_config,
            damage_seed,
        ],
    )

    dvl = Node(package='bluerov_stonefish', executable='dvl_to_twist_node', name='dvl_to_twist_node',
               output='screen', parameters=[{
                   'input_topic': '/bluerov/stonefish/sensors/dvl_sim',
                   'output_topic': '/bluerov/stonefish/sensors/dvl'}])
    battery = Node(package='bluerov_stonefish', executable='battery_status_simulated',
                   name='battery_status_simulated', output='screen', parameters=[{
                       'battery_topic': '/bluerov/stonefish/sensors/battery',
                       'thruster_topics': ['/bluerov/controller/thruster_setpoints_sim'],
                       'battery_frame_id': 'bluerov/battery'}])
    leak = Node(package='bluerov_stonefish', executable='leak_sensors_simulated', name='leak_sensors_simulated',
                output='screen', parameters=[{
                    'leak_topic': '/bluerov/stonefish/sensors/leak',
                    'sensor_frames': ['bluerov/main_cylinder', 'bluerov/battery_cylinder'],
                    'leak_detected': [False, False]}])

    return LaunchDescription([
        DeclareLaunchArgument('scenario',
                              default_value=PathJoinSubstitution([share, 'scenarios', 'bluerov_cirtesu_net.scn'])),
        DeclareLaunchArgument('simulation_data',
                              default_value=PathJoinSubstitution([FindPackageShare('bluerov_stonefish'), 'data'])),
        DeclareLaunchArgument('net_config', default_value=PathJoinSubstitution([share, 'config', 'net.yaml'])),
        DeclareLaunchArgument('simulation_rate', default_value='100.0'),
        DeclareLaunchArgument('window_width', default_value='1280'),
        DeclareLaunchArgument('window_height', default_value='720'),
        DeclareLaunchArgument('damage_seed', default_value='-1'),
        # sin sombras por defecto: los hilos de 2,5 mm parpadean en el mapa de sombras de Stonefish
        DeclareLaunchArgument('rendering_quality', default_value='noshadows'),
        simulator, dvl, battery, leak,
    ])
